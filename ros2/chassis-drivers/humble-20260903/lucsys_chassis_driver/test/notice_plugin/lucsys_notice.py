# Copyright 2026 LucSys Robotics - integration and maintenance changes only.
#
# LucSys Robotics distributes this source under supplier authorization.
# LucSys Robotics is responsible for integration and maintenance only.
# This notice does not grant an additional open-source license.
# Third-party components retain their respective licenses and notices.

"""Register the actual distribution notice with ament's standard license registry."""
from ament_copyright.licenses import LicenseEntryPoint

NOTICE = """LucSys Robotics distributes this source under supplier authorization.
LucSys Robotics is responsible for integration and maintenance only.
This notice does not grant an additional open-source license.
Third-party components retain their respective licenses and notices.
"""


supplier_authorization = LicenseEntryPoint(
    name='LucSys supplier-authorized distribution and maintenance notice',
    spdx='LicenseRef-LucSys-Supplier-Authorization',
    file_headers=['{copyright}\n\n' + NOTICE],
    license_files=[],
    contributing_files=[],
)
