from setuptools import find_packages, setup

package_name = 'jetcobot_movetag'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # 패키지 설명서
        ('share/' + package_name, ['package.xml']),
        # launch 파일 등록
        ('share/' + package_name + '/launch', ['launch/real_tf_broadcaster.launch.py',
                                                'launch/apriltag_move.launch.py',
                                                'launch/apriltag_flask.launch.py',
                                            ]),
    # # scripts (노드) 등록
        # ('share/' + package_name + '/scripts', ['scripts/apriltag_to_robot_node.py',
        #                                         'scripts/real_joint_state_publisher.py']),
        # urdf 폴더의 모든 파일 등록 (필요하다면)
        # ('share/' + package_name + '/urdf', ['urdf/여러 파일명']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eunyoung Park',
    maintainer_email='eunyoung927@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'real_joint_state_publisher = jetcobot_movetag.real_joint_state_publisher:main',
            'apriltag_move_node = jetcobot_movetag.apriltag_move_node:main',
            'apriltag_flask_node = jetcobot_movetag.apriltag_flask_node:main',  # Flask 서버 실행
        ],
    },
)
