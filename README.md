# VePack.Simulator

![sim.png](/sim.png)

### Requirements

* VeBotsメンバーであること
* .NET7 (SDK/Runtime)

### Installation

* AirSim ...  
1. Python パッケージ
```
pip install msgpack-rpc-python
pip install airsim
```
2. [AirSim ダウンロードページ](https://github.com/Microsoft/AirSim/releases) のAssetsから好きなzipファイルを落として展開。
3. exeを開いてみる。おそらくDirectXについてのエラーが出るので、[DirectX エンドユーザーランタイムを手動でインストールする方法](https://faq.tsukumo.co.jp/index.php?solution_id=1321)で解決。
4. [settings.json](/AirSim/settings.json) を展開したフォルダのexeがあるところに配置。

* VePack.Simulator ... 本リポジトリをクローン。
* VePack ... 
1. 最新版は私のプライベートリポジトリから。aresにも一応置いてるのでcollabolatorでない場合はそちらから。
2. VePack.Simulator.slnを開き、AirSim.csprojのプロジェクト参照を一度消してからVePack.csprojを追加する。これでソリューションを跨いだ参照ができる。
3. AirSim.csprojのビルドが通ることを確認する。失敗する場合はおそらく参照設定がうまくいっていない。


### Usage

1. まずAirSimのexeを実行。画面が立ち上がり、プログラムからの制御待ち状態になる。
2. 本プログラムを実行。ConsoleAppはCUI、WpfAppはGUIというだけで、どちらを実行してもよい。GrpcClientはサーバーと通信して遠隔監視される用のモック。

### About VePack
* Utilities ... IO とか Geometry とか
* Connectors ... sender / receiver のベースクラスとセンサ系
* Plugin/Navigation ... 位置方位をもとにマップ系の情報を返してくれる
* Plugin/Controllers ... 制御アルゴリズムの実装
* Plugin/Filters ... センサの後処理向けのフィルタ類
