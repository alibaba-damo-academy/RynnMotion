#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


V21_MESSAGE = (
    "The dataset you requested ({repo_id}) is in {version} format. "
    "We introduced a new format since v3.0 which is not backward compatible with v2.1. "
    "Please update your dataset using the conversion script."
)


class BackwardCompatibilityError(Exception):
    """Error raised when trying to use incompatible older datasets."""

    pass


class ForwardCompatibilityError(Exception):
    """Error raised when trying to use newer datasets with older codebase."""

    pass
