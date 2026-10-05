plugins {
    alias(libs.plugins.oist.application)
    alias(libs.plugins.chaquopy)
}

android {
    namespace = "jp.oist.abcvlib.dreamerBridge"

    buildFeatures {
        viewBinding = true
    }

    defaultConfig {
        ndk {
            // 64-bit only: Chaquopy offers Python 3.13 for arm64-v8a and
            // x86_64 alone, and the robot phone (Pixel 3a) is arm64. Dropping
            // the 32-bit ABIs also roughly halves the APK.
            abiFilters += listOf("arm64-v8a", "x86_64")
        }
    }
}

// The DreamerV3 policy runs on the phone, not on the trainer, and it is a
// numpy port of the JAX acting path (see dreamerv3/deploy/numpy_policy.py in
// the dreamerv3 repo). Chaquopy ships its own numpy wheels built for Android,
// so this is the whole dependency.
chaquopy {
    defaultConfig {
        // Chaquopy 17 requires buildPython to match the app's Python minor
        // version exactly, and 3.13 is what this Mac has. The default would be
        // 3.10, which is not installed here.
        version = "3.13"
        pip {
            install("numpy")
        }
    }
}
