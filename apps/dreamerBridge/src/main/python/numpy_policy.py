"""The DreamerV3 policy step in pure numpy, for running on the robot phone.

This is a port of the policy half of `dreamerv3/agent.py` -- encoder, RSSM
observe, actor head -- with no JAX, no ninjax and no repo imports. It is meant
to be copied verbatim into the phone app (Chaquopy already runs Python there),
so it must stay importable with nothing but numpy.

Why a port rather than a converter: TFLite/LiteRT bakes weights into the model
file at conversion time, so every weight update from the learner would mean
reconverting and shipping a new model. Here the weights are a plain .npz that
`load` swaps in place, which is exactly the update path the bridge already has.

The scope is deliberately the *acting* path only. There is no imagination, no
decoder, no value head -- the learner keeps all of that.

`tools/export_policy.py` writes the .npz this reads, and
`tools/test_numpy_policy.py` checks the output against the JAX agent step for
step. Any edit here must keep that test passing: a silent divergence between
this and the learner's own policy would show up only as a robot that acts
subtly worse than its training curve claims.
"""

import json

import numpy as np

__all__ = ['NumpyPolicy']

F32 = np.float32


# --- primitives, matching embodied/jax/nets.py -----------------------------


def silu(x):
  # jax.nn.silu. Computed in float32; the exp is the only place a float16
  # port would lose accuracy, and we do not have one.
  return x / (1.0 + np.exp(-x, dtype=F32))


def sigmoid(x):
  return 1.0 / (1.0 + np.exp(-x, dtype=F32))


def symlog(x):
  return np.sign(x) * np.log1p(np.abs(x))


def rms_norm(x, scale, eps=1e-4):
  """nets.Norm(impl='rms'). Always in float32, as the JAX version is."""
  x = x.astype(F32)
  mean2 = np.mean(np.square(x), axis=-1, keepdims=True)
  return x * (scale / np.sqrt(mean2 + eps))


def linear(x, kernel, bias=None):
  y = x @ kernel
  return y if bias is None else y + bias


def block_linear(x, kernel, bias, blocks):
  """nets.BlockLinear: a grouped matmul, einsum '...ki,kio->...ko'.

  Written as a batched matmul rather than einsum on purpose. einsum picks a
  generic loop path for this contraction, which on the phone's numpy is not
  BLAS-backed; `matmul` on a (blocks, 1, in) x (blocks, in, out) pair dispatches
  to a batched gemm instead. Same arithmetic, and it is the single hottest
  operation in the policy -- dynhid0 alone is 425k multiply-accumulates.
  """
  insize = x.shape[-1]
  x = x.reshape(*x.shape[:-1], blocks, insize // blocks)
  y = np.matmul(x[..., :, None, :], kernel)[..., :, 0, :]
  y = y.reshape(*y.shape[:-2], -1)
  return y + bias


def softmax(x, axis=-1):
  x = x - x.max(axis=axis, keepdims=True)
  e = np.exp(x)
  return e / e.sum(axis=axis, keepdims=True)


class NumpyPolicy:
  """One acting step of the agent, carrying its own RSSM state.

  Usage on the phone:

      pol = NumpyPolicy('policy.npz')
      carry = pol.initial()
      # then, once per control tick:
      act, carry = pol.step({'orientation': ..., 'wheels': ...}, carry,
                            is_first=False)
  """

  def __init__(self, path=None, seed=0):
    self.rng = np.random.default_rng(seed)
    self.w = {}
    self.meta = {}
    self.nonfinite = 0  # actions refused by the finite check in _actor
    if path is not None:
      self.load(path)

  # -- weights ------------------------------------------------------------

  def load(self, path):
    """Swap in a new weight set. Safe to call between steps; the RSSM carry
    stays valid because only the parameters change, not the state layout."""
    with np.load(path, allow_pickle=False) as data:
      meta = json.loads(str(data['__meta__'].item()))
      w = {k: np.asarray(data[k], F32) for k in data.files if k != '__meta__'}
    if self.meta and meta['signature'] != self.meta['signature']:
      raise ValueError(
          'Refusing to load weights with a different architecture: '
          f"{meta['signature']} != {self.meta['signature']}")
    self.meta, self.w = meta, w
    return meta

  # -- state --------------------------------------------------------------

  def initial(self):
    m = self.meta
    return dict(
        deter=np.zeros(m['deter'], F32),
        stoch=np.zeros((m['stoch'], m['classes']), F32),
        prevact=np.zeros(m['act_dim'], F32))

  # -- the step -----------------------------------------------------------

  def step(self, obs, carry, is_first=False, mode='train'):
    """Return (action, carry). `obs` maps each vector key to its raw array.

    `mode='train'` samples, matching what the actor does while collecting;
    `mode='eval'` takes the distribution mode, which is what you want for a
    demo run where exploration noise is not wanted.
    """
    return self.finish(obs, carry, self.precompute(carry, is_first), mode)

  def precompute(self, carry, is_first=False):
    """The half of the step that does not need the observation.

    The GRU advances the deterministic state from the previous carry and the
    previous action alone, and it is about half the arithmetic and half the
    weights streamed (dynhid0 is the largest kernel in the policy). On the
    robot the observation arrives at a known moment and the action is due a
    few milliseconds later, so this runs *before* it arrives, right after the
    previous step while the core is still warm, and only `finish` is on the
    sensor-to-torque path.
    """
    deter, stoch, prevact = carry['deter'], carry['stoch'], carry['prevact']
    if is_first:
      # nn.mask(..., ~reset): the carry and the previous action are zeroed on
      # the first step of an episode rather than carried across the boundary.
      deter = np.zeros_like(deter)
      stoch = np.zeros_like(stoch)
      prevact = np.zeros_like(prevact)
    return self._core(deter, stoch, prevact)

  def finish(self, obs, carry, deter, mode='train'):
    """The observation-dependent half: encode, posterior, actor.

    `deter` is what `precompute` returned for this same `carry`.
    """
    tokens = self._encode(obs)
    stoch, _ = self._obs_stoch(deter, tokens)
    act = self._actor(deter, stoch, mode)
    carry = dict(deter=deter, stoch=stoch, prevact=act)
    return act, carry

  # -- pieces -------------------------------------------------------------

  def _encode(self, obs):
    w, m = self.w, self.meta
    # DictConcat sorts its keys, so the encoder input order is the sorted
    # observation key order, not the order the caller happens to use.
    parts = [np.asarray(obs[k], F32).reshape(-1) for k in m['obs_keys']]
    x = np.concatenate(parts, -1)
    if m['symlog']:
      x = symlog(x)
    for i in range(m['enc_layers']):
      x = linear(x, w[f'enc/mlp{i}/kernel'], w[f'enc/mlp{i}/bias'])
      x = silu(rms_norm(x, w[f'enc/mlp{i}norm/scale']))
    return x

  def _core(self, deter, stoch, action):
    """RSSM._core: the blocked GRU that advances the deterministic state."""
    w, m = self.w, self.meta
    g = m['blocks']
    stoch = stoch.reshape(-1)
    # action /= max(1, |action|), elementwise -- keeps an out-of-range action
    # from blowing up the recurrence.
    action = action / np.maximum(1.0, np.abs(action))

    x0 = linear(deter, w['dyn/dynin0/kernel'], w['dyn/dynin0/bias'])
    x0 = silu(rms_norm(x0, w['dyn/dynin0norm/scale']))
    x1 = linear(stoch, w['dyn/dynin1/kernel'], w['dyn/dynin1/bias'])
    x1 = silu(rms_norm(x1, w['dyn/dynin1norm/scale']))
    x2 = linear(action, w['dyn/dynin2/kernel'], w['dyn/dynin2/bias'])
    x2 = silu(rms_norm(x2, w['dyn/dynin2norm/scale']))

    x = np.concatenate([x0, x1, x2], -1)          # (3*hidden,)
    x = np.repeat(x[None, :], g, axis=0)           # (g, 3*hidden)
    x = np.concatenate([deter.reshape(g, -1), x], -1).reshape(-1)

    for i in range(m['dynlayers']):
      x = block_linear(
          x, w[f'dyn/dynhid{i}/kernel'], w[f'dyn/dynhid{i}/bias'], g)
      x = silu(rms_norm(x, w[f'dyn/dynhid{i}norm/scale']))

    x = block_linear(x, w['dyn/dyngru/kernel'], w['dyn/dyngru/bias'], g)
    # Gates are split within each block, then flattened back.
    gates = x.reshape(g, -1)
    reset, cand, update = np.split(gates, 3, axis=-1)
    reset = reset.reshape(-1)
    cand = cand.reshape(-1)
    update = update.reshape(-1)
    reset = sigmoid(reset)
    cand = np.tanh(reset * cand)
    # The -1 bias makes the gate start closed, so the state persists by
    # default rather than being overwritten each step.
    update = sigmoid(update - 1.0)
    return update * cand + (1 - update) * deter

  def _obs_stoch(self, deter, tokens):
    """Posterior over the stochastic state given the new observation."""
    w, m = self.w, self.meta
    x = np.concatenate([deter, tokens], -1)
    for i in range(m['obslayers']):
      x = linear(x, w[f'dyn/obs{i}/kernel'], w[f'dyn/obs{i}/bias'])
      x = silu(rms_norm(x, w[f'dyn/obs{i}norm/scale']))
    logit = linear(x, w['dyn/obslogit/kernel'], w['dyn/obslogit/bias'])
    logit = logit.reshape(m['stoch'], m['classes'])
    return self._sample_onehot(logit), logit

  def _sample_onehot(self, logit):
    unimix = self.meta['unimix']
    probs = softmax(logit.astype(F32), -1)
    if unimix:
      probs = (1 - unimix) * probs + unimix / probs.shape[-1]
    # Per-row categorical draw, vectorised: one uniform per row against the
    # row's cumulative distribution.
    cdf = np.cumsum(probs, -1)
    u = self.rng.random((probs.shape[0], 1), dtype=np.float64)
    idx = (u > cdf).sum(-1)
    idx = np.minimum(idx, probs.shape[-1] - 1)
    out = np.zeros_like(probs)
    out[np.arange(probs.shape[0]), idx] = 1.0
    return out.astype(F32)

  def _actor(self, deter, stoch, mode):
    w, m = self.w, self.meta
    x = np.concatenate([deter, stoch.reshape(-1)], -1)
    for i in range(m['pol_layers']):
      x = linear(x, w[f'pol/mlp/linear{i}/kernel'], w[f'pol/mlp/linear{i}/bias'])
      x = silu(rms_norm(x, w[f'pol/mlp/norm{i}/scale']))
    key = m['act_key']
    mean = linear(
        x, w[f'pol/head/{key}/mean/kernel'], w[f'pol/head/{key}/mean/bias'])
    raw = linear(
        x, w[f'pol/head/{key}/stddev/kernel'], w[f'pol/head/{key}/stddev/bias'])
    lo, hi = m['minstd'], m['maxstd']
    stddev = (hi - lo) * sigmoid(raw + 2.0) + lo
    mean = np.tanh(mean)
    if mode == 'eval':
      act = mean
    else:
      act = mean + stddev * self.rng.standard_normal(mean.shape).astype(F32)
    # A non-finite action would go straight to the wheels as a garbage PWM
    # value, so it is worth the two microseconds to refuse it. This has never
    # fired in testing; it is here because the failure it prevents is physical.
    # (Note: numpy on macOS/Accelerate raises spurious divide-by-zero and
    # overflow flags on perfectly finite matmuls, so trust this check rather
    # than np.seterr warnings when debugging on the Mac.)
    if not np.isfinite(act).all():
      self.nonfinite += 1
      return np.zeros_like(act)
    # The env clips before it reaches the wheels; do it here so the action the
    # phone stores in replay is the action it actually applied.
    return np.clip(act, -1.0, 1.0).astype(F32)
