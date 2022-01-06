# VePack.Simulator

![sim.png](/sim.png)

### Requirements

* VeBots member
* .NET6 development environment

### Installation

* VePack ... contact me.
* AirSim ...  
1. Install PyPI packages
```
pip install msgpack-rpc-python
pip install airsim
pip install pynput
```
2. Go to [AirSim download binaries page](https://github.com/Microsoft/AirSim/releases) and download your preference zip file from assets.
3. Open .exe file. If you face error about DirectX, see [How to install DirectX end user runtime](https://faq.tsukumo.co.jp/index.php?solution_id=1321).
4. Set [settings.json](/AirSim/settings.json) at the same folder as .exe.

### About VePack
* Utilities ... utilities such as IO and Geometry
* Connectors ... sensor implementation and base class of sender / receiver
* Plugin/Navigation ... serve map-base information on given position and direction
* Plugin/Controllers ... implementation of vehicle control module
* Plugin/Filters ... for post-process of sensor's output
