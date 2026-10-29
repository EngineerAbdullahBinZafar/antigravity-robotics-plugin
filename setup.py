from setuptools import find_packages, setup

setup(
    name="antigravity-robotics-plugin",
    version="1.2.0",
    author="Antigravity Robotics Team",
    author_email="robotics@agent-plugins.org",
    description="Native Antigravity Plugin packaging ROS 2 MCP Server with interactive Web Dashboard.",
    long_description=open("README.md", encoding="utf-8").read()
    if open("README.md").readable()
    else "",
    long_description_content_type="text/markdown",
    packages=find_packages(include=["ros2_mcp*"]),
    python_requires=">=3.10",
    install_requires=[],
    extras_require={
        "rosbridge": ["websocket-client>=1.6.0"],
        "dashboard": ["aiohttp>=3.9.0"],
        "dev": ["pytest>=7.4", "pytest-cov>=4.0", "ruff>=0.4"],
    },
    entry_points={
        "console_scripts": [
            "ros2-mcp = ros2_mcp.server:main",
            "ros2-mcp-server = ros2_mcp.server:main",
            "antigravity-robotics = ros2_mcp.server:main",
            "ros2-mcp-dashboard = ros2_mcp.dashboard.server:main",
        ],
    },
    package_data={
        "ros2_mcp.dashboard": ["static/*", "static/**/*"],
    },
    include_package_data=True,
    classifiers=[
        "Development Status :: 5 - Production/Stable",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Operating System :: POSIX :: Linux",
        "Operating System :: Microsoft :: Windows",
        "Operating System :: MacOS",
    ],
)
