# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES.
# SPDX-License-Identifier: Apache-2.0

# rules_python does not process cmeel.pth; load its bundled path bootstrap.
import cmeel_pth  # noqa: F401
