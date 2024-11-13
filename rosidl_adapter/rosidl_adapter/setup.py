from setuptools import setup, find_packages

setup(
    name='rosidl_adapter',
    version='4.9.0',  # Version from CHANGELOG.rst
    packages=find_packages(),
    package_data={
        'rosidl_adapter': [
            'py.typed',
            'resource/**/*',  # Include all files under resource directory
        ],
    },
    install_requires=[
        'catkin_pkg',
        'empy',
        'rosidl_cli',
    ],
    entry_points={
        'rosidl_cli.command.translate.extensions': [
            'msg2idl = rosidl_adapter.cli:TranslateMsgToIDL',
            'srv2idl = rosidl_adapter.cli:TranslateSrvToIDL',
            'action2idl = rosidl_adapter.cli:TranslateActionToIDL',
        ],
    },
    scripts=[
        'scripts/msg2idl.py',
        'scripts/srv2idl.py',
        'scripts/action2idl.py',
    ],
    author='Open Robotics',
    author_email='info@openrobotics.org',
    description='API and scripts to parse .msg/.srv/.action files and convert them to .idl',
    license='Apache License 2.0',
    python_requires='>=3.6',
) 