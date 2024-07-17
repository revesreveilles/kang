# 解决Isaac sim&&lab运行示例时isaacsim等待时间过长的问题
## 部署NUCLEUS
1. 在`Omniverse Launcher`中搜索安装`Nucleus Navigator`
   - 在`Nucleus`中完成本地服务器的创建与设置
2. 在`EXCHANGE`中搜索`isaac`，安装所有的`ISAAC SIM ASSETS PACK` 
   - 安装后将资产包2、3、4的内容合并至资产包**1**(ISAAC SIM ASSETS PACK 1 4.0.0)中
3. 运行`Nucleus Navigatior`,进入`localhost`-->`Library`，创建如下子级目录：`omniverse://localhost/Library/NVIDIA/Assets/Isaac/`
   - 将合并后的资产包**1**内的`4.0`拖入当前目录下 
4. 修改isaacsim的默认资产路径:
```
cd ~/.local/share/ov/pkg/isaac-sim-4.0.0
./isaac-sim.sh --/persistent/isaac/asset_root/default="omniverse://localhost/Library/NVIDIA/Assets/Isaac/4.0"
```
   - 在isaacsim中查看是否更改路径成功:`Isaac Utils`-->`Nucleus Check`，出现以下输出则说明更改成功:
    ```
    [139.213s] Checking for Isaac Sim assets...
    [139.218s] Isaac Sim assets found: omniverse://localhost/Library/NVIDIA/Assets/Isaac/4.0
    ```
## 修改后运行Isaac lab 仍出现该情况的解决方案:
### 原因：Isaac Lab启动isaac sim的时候修改了 "/persistent/isaac/asset_root/default"
```
cd /home/<username>/IsaacLab/source/extensions/omni.isaac.lab/omni/isaac/lab/app
vim app_launcher.py
```
找到`assets_path`，修改为：`omniverse://localhost/Library/NVIDIA/Assets/Isaac/4.0`
