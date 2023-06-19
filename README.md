# ROS2パッケージの開発

# ROS2パッケージの開発環境の準備

開発環境を整える必要がある。

- ROS2の環境が入っている
- ビルドに必要なライブラリが入っている


```docker
docker-compose build
docker-compose run -it ros2 bash
```

で起動して接続する。

# ROS2 パッケージの生成

以下のコマンドで生成できる。

```docker
cd src
ros2 pkg create --build-type ament_python <package-name>
```

ワラワラとたくさんファイルが生成される。

# gitignoreの設定

以下のファイル群を設定しておくと良さそう。

```docker
log/
install/
build/

*.buildinfo
*.changes
*.deb
*.cfg
debhelper-build-stamp
files
*.egg-info/
dist-packages/
usr/
DEBIAN/
```

# ROS2 ノードの作成

```docker
ros_workspace/src/<package-name>/<package-name>/simple_node.py

import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Node is started')

def main(args=None):
    rclpy.init(args=args)

    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

# ROS2 ノードの実行

setup.pyを編集する必要がある。赤字の部分が重要

```docker
from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_node = py_pubsub.simple_node:main',
        ],
    },
)
```

以下コマンドで実行できる

```docker
ros2 run <package-name> <script名>
```

上に乗せたscriptだと、Node is startedとログをはいて、rclpy.spin(node) する。

rclpy.spinは、ノードが終了するまで無限ループし、メッセージの受信とコールバックの実行を継続する。

# ROS2パッケージのビルド

パッケージのビルドは複数の手段がある様です。

- colcanを使用する
- dpkg-buildpackageを使用する

dpkg-buildpackageでビルドするにはいくつかファイルの用意が必要です。

**1. `debian/` ディレクトリの作成**

```
├── Dockerfile
├── docker-compose.yml
└── ros_workspace
    └── src
        └── py_pubsub
            ├── debian
            │   ├── changelog
            │   ├── control
            │   ├── py-pubsub
            │   └── rules
            ├── setup.py
```

**2. `debian/control` ファイルの作成**

```docker
Source: <package-name>
Section: misc
Priority: optional
Maintainer: <Your Name> <<your-email@example.com>>
Build-Depends: debhelper-compat (= 12), python3 (>= 3.6), dh-python
Standards-Version: 3.9.8

Package: <package-name>
Architecture: any
Depends: ${shlibs:Depends}, ${misc:Depends}
Description: <description of your package>
```

**3. `debian/rules` ファイルの作成**

```docker
#!/usr/bin/make -f

%:
	dh $@ --with python3 --buildsystem=pybuild
```

```docker
chmod +x debian/rules
```

**4. `debian/changelog` ファイルの作成**

dchで生成する。dchについては下にまとめる。こんなフォーマットで記載されていること

(手動で書いても良さそう)

```docker
(例)
py-pubsub (2.0.5) UNRELEASED; urgency=medium

  * Describe changes made in this version.

 -- ratovia <ratovia@hotmail.co.jp>  Mon, 19 Jun 2023 05:17:12 +0000

py-pubsub (2.0.4) jammy; urgency=medium

  *
  * Describe changes made in this version.
  * Describe changes made in this version.

 -- ratovia <ratovia@hotmail.co.jp>  Mon, 19 Jun 2023 05:16:58 +0000
```

全て完了したら以下のコマンドで作成

```docker
dpkg-buildpackage -us -uc -b
```

ビルドファイルがめちゃくちゃ発生するがgitignoreに設定しておけば大丈夫

# dchについて

dchでパッケージのバージョンをコントロールできる。

以下のコマンドでchangelogファイルの作成ができる。(既存システムには使用しない)

```docker
dch --create -v 1.0.0 --package <パッケージ名>
```

既存のchangelogにバージョンエントリを追加するには

```docker
dch -v 1.0.1 "Some changes for the new version."
```

しかし、バージョンエントリが追加されるのは最新のchangelogがUNRELEASEDではない時のみ。

```docker
py-pubsub (2.0.5) UNRELEASED; urgency=medium

  * Describe changes made in this version.

 -- ratovia <ratovia@hotmail.co.jp>  Mon, 19 Jun 2023 05:17:12 +0000

py-pubsub (2.0.4) jammy; urgency=medium

  *
  * Describe changes made in this version.
  * Describe changes made in this version.

 -- ratovia <ratovia@hotmail.co.jp>  Mon, 19 Jun 2023 05:16:58 +0000
```

これをreleaseにするには以下のコマンド

```docker
dch --release
```
