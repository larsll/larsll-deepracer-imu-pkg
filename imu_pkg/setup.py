#################################################################################
#   Copyright Lars Ludvigsen.                     All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

from setuptools import setup
import os

package_name = "imu_pkg"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), ["launch/imu_pkg_launch.py"])
    ],
    install_requires=["setuptools","BMI160-i2c","smbus2"],
    zip_safe=True,
    maintainer="Lars Ludvigsen",
    maintainer_email="lars@ludvig.no",
    description="This package contains the IMU node that publishes messages from the Inertial Measurement Unit.",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "imu_node = imu_pkg.imu_node:main"
        ],
    },
)
