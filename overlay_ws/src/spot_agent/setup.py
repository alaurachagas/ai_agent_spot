from setuptools import find_packages, setup

package_name = 'spot_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/spot_agent.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ana Laura Soethe Chagas',
    maintainer_email='ana.soethe.chagas@wzl-iqs.rwth-aachen.de',
    description='AI agent ROS 2 package with separate tool nodes.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent = spot_agent.agent.agent_node:main'
            #'tool_a = spot_agent.tools.tool_a_node:main'
            #'tool_b = spot_agent.tools.tool_b_node:main'

        ],
    },
)
