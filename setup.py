from setuptools import setup, find_packages

setup(
    name="LouCOMAX_Controllers",
    version="0.1.0",
    description="Controllers for all devices used in the LouCOMAX (CNRS) project",
    author="BLASIAK Antoine",
    author_email="antoineblasiak66@gmail.com",
    packages=find_packages(),
    install_requires=[
        "numpy",
        "overrides",
        "clr",
        "pyusb",
        "pyserial",
        "PyQt5",
        "csu2controller",
        "zaber_motion",
    ],
    python_requires=">=3.8",
    include_package_data=True,
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
)