# Copyright (c) Facebook, Inc. and its affiliates.
#
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
# Patched error with any dirs inside python/scripts
import os
from setuptools import setup
from setuptools import find_packages
from setuptools import find_namespace_packages

script_dir = "python/scripts"
scripts = [
    os.path.join(script_dir, f)
    for f in os.listdir(script_dir)
    if os.path.isfile(os.path.join(script_dir, f))
]

packages = find_packages(where="python") + find_namespace_packages(
    include=["hydra_plugins.*"], where="python"
)

setup(
    name="polymetis",
    version="0.2",
    packages=packages,
    package_dir={"": "python"},
    include_package_data=True,
    scripts=scripts,
    install_requires=["alephzero"],
)