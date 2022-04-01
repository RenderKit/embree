# Copyright 2009-2021 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def embree_deps():
    """Fetches third party depencendies of Embree"""

    maybe(
        git_repository,
        name = "oneTBB",
        commit = "3e352b48127b3c79d61df5618607d2daab3f2caa",
        shallow_since = "1648206148 +0300",
        remote = "https://github.com/oneapi-src/oneTBB",
    )
