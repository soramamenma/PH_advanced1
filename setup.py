from glob import glob
from setuptools import find_packages, setup

setup(
    name="pai_ai_navigation", version="0.1.0", packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/pai_ai_navigation"]),
        ("share/pai_ai_navigation", ["package.xml"]),
        ("share/pai_ai_navigation/launch", glob("launch/*.launch.py")),
        ("share/pai_ai_navigation/config", glob("config/*.yaml")),
        ("share/pai_ai_navigation/worlds", glob("worlds/*.sdf")),
        ("share/pai_ai_navigation/models", glob("models/*.pt") + glob("models/*.json")),
    ],
    package_data={"pai_ai_navigation": ["dashboard.html"]},
    install_requires=["setuptools", "numpy", "torch"], zip_safe=True,
    maintainer="moyashimame", maintainer_email="noreply@example.invalid",
    description="ROS 2 navigation learning experiment", license="Apache-2.0",
    entry_points={"console_scripts": ["probe = pai_ai_navigation.probe:main", "controller = pai_ai_navigation.controller:main", "shield = pai_ai_navigation.shield:main", "episode = pai_ai_navigation.episode:main", "train = pai_ai_navigation.train:main"]},
)
