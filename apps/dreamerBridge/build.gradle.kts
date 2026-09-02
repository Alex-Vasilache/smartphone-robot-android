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
            abiFilters += listOf("arm64-v8a", "armeabi-v7a", "x86_64", "x86")
        }
    }
}
