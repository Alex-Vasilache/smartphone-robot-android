"""Emotion signals from the world model, in numpy: valence and arousal.

Runs beside `numpy_policy.NumpyPolicy` (same weights file, which then also
needs the RSSM prior and the critic and reward heads: export with
`emotion=True`), on the phone for its face and offline on recorded runs
(`tools/robot/dev/emotion_trace.py`). Per step, with the posterior q_t the
policy just formed from the observation and the prior p_t the world model
predicted for it:

  valence     TD error r_t + gamma V(s_t) - V(s_{t-1}) (Schultz et al. 1997):
              better or worse than the critic expected
  surprise    KL(q_t || p_t), nats: how much the observation moved the belief
  novelty     H(p_t): how uncertain the prediction was before it
  unexpected  CE(q_t, p_t) - H(p_t) = H(q_t) + KL - H(p_t): the surprise beyond
              what the prediction's own spread expected (Yu & Dayan 2005's
              unexpected uncertainty; static noise scores ~0)
  reward_err  |r_t - r_hat(s_t)|: reward prediction error
  empowerment one-step-to-n-step channel proxy: roll K fixed action sequences
              n steps through the prior from s_t and take the mean KL of each
              end prediction from their mixture (per latent, summed): how much
              what the robot does changes where it ends up (Klyubin et al. 2005)

and from those, normalised online (running mean and SD, `tau` steps):

  arousal = sigmoid(w_n z(novelty) + w_u z(unexpected) + w_r z(reward_err)
                    - w_e z(empowerment) + w_h z(|TD|) * (1 - control))
  valence = tanh(TD error / its running RMS)        joy / distress, brief
  mood    = tanh(z(V(s_t)), slow)                   hope / fear (Broekens 2015)
  dominance = tanh(z(empowerment))

The last arousal term is helplessness (Maier & Seligman 2016): large errors
with little control, control = sigmoid(z(empowerment)). With a dense reward
the TD error sits near 0 most of the time, so the face shows `face_valence`,
mostly mood with the TD error's flashes on top.

Arousal rises with novelty and unexpected surprise and falls with control;
low control with large errors is the high-arousal corner either way, as
in helplessness or gambling.
"""

import numpy as np

from numpy_policy import F32, linear, rms_norm, silu, sigmoid, softmax  # noqa: E402  (flat layout on the phone)

# Empowerment's action set: the 3x3 grid of constant wheel commands.
ACTIONS = np.array([(l, r) for l in (-1.0, 0.0, 1.0) for r in (-1.0, 0.0, 1.0)],
                   F32)


def _probs(logit, unimix):
  p = softmax(np.asarray(logit, F32), -1)
  if unimix:
    p = (1 - unimix) * p + unimix / p.shape[-1]
  return p


def _entropy(p):
  return float(-(p * np.log(p + 1e-8)).sum())


def _kl(p, q):
  return float((p * (np.log(p + 1e-8) - np.log(q + 1e-8))).sum())


def _twohot_bins(n):
  half = np.linspace(-20, 0, (n - 1) // 2 + 1, dtype=np.float64)
  half = np.sign(half) * (np.exp(np.abs(half)) - 1)   # symexp
  return np.concatenate([half, -half[:-1][::-1]], 0).astype(F32)


class Running:
  """Exponential running mean and SD, for z-scores without a fixed scale."""

  def __init__(self, tau):
    self.a = 1.0 / float(tau)
    self.mean, self.var, self.n = 0.0, 1.0, 0

  def __call__(self, x):
    x = float(x)
    if self.n == 0:
      self.mean, self.var = x, 1e-6
    a = max(self.a, 1.0 / (self.n + 1))
    d = x - self.mean
    self.mean += a * d
    self.var = (1 - a) * (self.var + a * d * d)
    self.n += 1
    return d / np.sqrt(self.var + 1e-8)


class Emotion:

  def __init__(self, policy, gamma=1 - 1 / 130, horizon=8, every=1, tau=650,
               weights=(1.0, 1.0, 1.0, 1.0, 1.0)):
    """`policy` is a loaded NumpyPolicy whose file has the prior and heads.
    `every`: empowerment only every this many steps (it costs K x horizon
    GRU steps), held in between. `tau`: steps of the running normalisation
    (650 = 10 s at 65 Hz). `weights`: (novelty, unexpected, reward error,
    empowerment, helplessness) in the arousal sum."""
    self.p = policy
    missing = [k for k in ('dyn/priorlogit/kernel', 'val/head/logits/kernel',
                           'rew/head/logits/kernel') if k not in policy.w]
    if missing:
      raise ValueError(f'weights lack {missing}: export with emotion=True')
    self.gamma, self.horizon, self.every = float(gamma), int(horizon), int(every)
    self.w = np.asarray(weights, F32)
    self.norm = {k: Running(tau) for k in (
        'novelty', 'unexpected', 'reward_err', 'empowerment', 'td', 'abs_td')}
    self.mood_norm = Running(tau * 6)
    self.bins = _twohot_bins(policy.w['val/head/logits/bias'].shape[-1])
    self.reset()

  def reset(self):
    self.prev_value = None
    self.steps = 0
    self.empowerment = 0.0

  # -- network pieces the policy file does not use for acting --------------

  def prior_logit(self, deter):
    w, m = self.p.w, self.p.meta
    x = deter
    i = 0
    while f'dyn/prior{i}/kernel' in w:
      x = linear(x, w[f'dyn/prior{i}/kernel'], w[f'dyn/prior{i}/bias'])
      x = silu(rms_norm(x, w[f'dyn/prior{i}norm/scale']))
      i += 1
    logit = linear(x, w['dyn/priorlogit/kernel'], w['dyn/priorlogit/bias'])
    return logit.reshape(*deter.shape[:-1], m['stoch'], m['classes'])

  def head(self, name, deter, stoch):
    w = self.p.w
    x = np.concatenate([deter, stoch.reshape(*deter.shape[:-1], -1)], -1)
    i = 0
    while f'{name}/mlp/linear{i}/kernel' in w:
      x = linear(x, w[f'{name}/mlp/linear{i}/kernel'], w[f'{name}/mlp/linear{i}/bias'])
      x = silu(rms_norm(x, w[f'{name}/mlp/norm{i}/scale']))
      i += 1
    logits = linear(x, w[f'{name}/head/logits/kernel'], w[f'{name}/head/logits/bias'])
    return float((softmax(logits, -1) * self.bins).sum(-1))

  def _core_batch(self, deter, stoch, action):
    """NumpyPolicy._core over a leading batch axis."""
    w, m = self.p.w, self.p.meta
    g, B = m['blocks'], deter.shape[0]
    stoch = stoch.reshape(B, -1)
    action = action / np.maximum(1.0, np.abs(action))
    x0 = silu(rms_norm(linear(deter, w['dyn/dynin0/kernel'], w['dyn/dynin0/bias']), w['dyn/dynin0norm/scale']))
    x1 = silu(rms_norm(linear(stoch, w['dyn/dynin1/kernel'], w['dyn/dynin1/bias']), w['dyn/dynin1norm/scale']))
    x2 = silu(rms_norm(linear(action, w['dyn/dynin2/kernel'], w['dyn/dynin2/bias']), w['dyn/dynin2norm/scale']))
    x = np.concatenate([x0, x1, x2], -1)
    x = np.repeat(x[:, None, :], g, axis=1)
    x = np.concatenate([deter.reshape(B, g, -1), x], -1)          # (B, g, in)
    for i in range(m['dynlayers']):
      x = self._block(x, w[f'dyn/dynhid{i}/kernel'], w[f'dyn/dynhid{i}/bias'])
      x = silu(rms_norm(x.reshape(B, -1), w[f'dyn/dynhid{i}norm/scale'])).reshape(B, g, -1)
    x = self._block(x, w['dyn/dyngru/kernel'], w['dyn/dyngru/bias'])
    reset, cand, update = np.split(x, 3, axis=-1)
    reset, cand, update = (y.reshape(B, -1) for y in (reset, cand, update))
    cand = np.tanh(sigmoid(reset) * cand)
    update = sigmoid(update - 1.0)
    return update * cand + (1 - update) * deter

  @staticmethod
  def _block(x, kernel, bias):
    # x (B, g, in) with kernel (g, in, out): each block its own weights.
    y = np.einsum('bgi,gio->bgo', x, kernel)
    return y + bias.reshape(1, kernel.shape[0], -1)

  def _empowerment(self, deter, stoch):
    K, unimix = len(ACTIONS), self.p.meta['unimix']
    d = np.repeat(deter[None], K, 0)
    s = np.repeat(stoch[None], K, 0)
    for _ in range(self.horizon):
      d = self._core_batch(d, s, ACTIONS)
      p = _probs(self.prior_logit(d), unimix)            # (K, stoch, classes)
      s = np.eye(p.shape[-1], dtype=F32)[p.argmax(-1)]   # the likeliest latent
    mix = p.mean(0, keepdims=True)
    return float(np.mean([_kl(p[k], mix[0]) for k in range(K)]))

  # -- the step ---------------------------------------------------------------

  def step(self, deter, post_logit, stoch, reward, is_first=False, steps=1):
    """`deter`: the step's deterministic state (precompute's output);
    `post_logit`/`stoch`: the posterior logits and its sample; `reward`: the
    reward that came with this observation. Called every `steps` control
    steps (the phone), `reward` is their discounted sum since the last call.
    Returns a dict of signals. With every <= 0 empowerment is left to the
    caller (`self.empowerment`, e.g. from a thread)."""
    unimix = self.p.meta['unimix']
    if is_first:
      self.reset()
    q = _probs(post_logit, unimix)
    pr = _probs(self.prior_logit(deter), unimix)
    surprise = _kl(q, pr)
    novelty = _entropy(pr)
    unexpected = _entropy(q) + surprise - novelty
    value = self.head('val', deter, stoch)
    r_hat = self.head('rew', deter, stoch)
    reward_err = abs(float(reward) - r_hat)
    td = 0.0 if self.prev_value is None else (
        float(reward) + self.gamma ** steps * value - self.prev_value)
    self.prev_value = value
    if self.every > 0 and self.steps % self.every == 0:
      self.empowerment = self._empowerment(deter, stoch)
    self.steps += 1
    z = {k: self.norm[k](v) for k, v in (
        ('novelty', novelty), ('unexpected', unexpected),
        ('reward_err', reward_err), ('empowerment', self.empowerment))}
    # RMS, not SD: a steady disappointment (lying on a bumper) is a TD error
    # with a mean and little spread, and should read clearly negative, not
    # saturate at -1.
    a = max(self.norm['td'].a, 1.0 / (self.norm['td'].n + 1))
    self._td_ms = (1 - a) * getattr(self, '_td_ms', td * td) + a * td * td
    self.norm['td'].n += 1
    td_scale = np.sqrt(self._td_ms + 1e-8)
    control = float(sigmoid(np.float32(z['empowerment'])))
    helpless = self.norm['abs_td'](abs(td)) * (1 - control)
    drive = (self.w[0] * z['novelty'] + self.w[1] * z['unexpected']
             + self.w[2] * z['reward_err'] - self.w[3] * z['empowerment']
             + self.w[4] * helpless)
    valence = float(np.tanh(td / max(td_scale, 1e-6)))
    mood = float(np.tanh(self.mood_norm(value) / 2.0))
    return dict(
        valence=valence, mood=mood,
        face_valence=float(np.clip(0.7 * mood + 0.3 * valence, -1, 1)),
        helpless=float(helpless),
        arousal=float(sigmoid(np.float32(drive / 2.0))),
        dominance=float(np.tanh(z['empowerment'])),
        td=td, value=value, reward_hat=r_hat, surprise=surprise,
        novelty=novelty, unexpected=unexpected, reward_err=reward_err,
        empowerment=self.empowerment)
