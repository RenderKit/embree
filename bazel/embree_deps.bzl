# Copyright 2022 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def embree_deps():
    """Fetches third party depencendies of Embree"""

    maybe(
        git_repository,
        name = "oneTBB",
        commit = "3a7f96db56cc9821055cbc769d3065db86b8b4c9",
        shallow_since = "1644216975 +0300",
        remote = "https://github.com/oneapi-src/oneTBB",
    )
