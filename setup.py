from glob import glob

from setuptools import find_packages, setup


package_name = "featpose"

setup(
    name=package_name,
    version="0.1.0",
    description="A package containing some inference nodes and sample models for 6D pose estimation using feature detection.",
    author="Leonhart Root",
    author_email="root.link.sky@gmail.com",
    maintainer="Leonhart Root",
    maintainer_email="root.link.sky@gmail.com",
    url="https://github.com/verithm/featpose",
    packages=find_packages(where="src", exclude=["test"]),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3 :: Only",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Programming Language :: Python :: 3.13",
        "Topic :: Scientific/Engineering",
        "Topic :: Software Development",
    ],
    license="Apache-2.0",
    keywords="feature, pose, inference",
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/model", glob("model/*.npz")),
    ],
    package_dir={"": "src"},
    zip_safe=True,
    install_requires=[
        "setuptools",
        "numpy",
        "opencv-python",
        "scipy",
    ],
    entry_points={
        "console_scripts": [
            "featpose_webcam = featpose.featpose_webcam:main",
        ],
    },
    python_requires=">=3.10",
    tests_require=["pytest"],
    project_urls={
        "Bug Reports": "https://github.com/verithm/featpose/issues",
        "Source": "https://github.com/verithm/featpose/",
    },
)
