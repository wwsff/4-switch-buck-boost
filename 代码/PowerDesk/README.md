# PowerDesk发布包

Copyright (c) 2026 SZPUW — 原创代码GPL-3.0-only。

这是桌面电源项目的电脑上位机。安装Python（原说明要求3.11及以上）后，在本目录运行：

```powershell
python -m pip install -r requirements.txt
python server.py
```

也可双击Start_PowerDesk.bat。演示模式不连接真实设备；真实串口115200/8N1。G474自动准备零点；H750/CAN接管后，本上位机主要用于读取遥测和诊断，多数写命令会返回BUSY。首次运行没有作者设备档案，profiles.json仅在使用过程中按自己的设备生成。

完整操作、配套版本、已测范围与限制见整个开源目录README_开源使用说明.md。旧验证文档不随精简包分发，软件测试源码保留在tests/。默认缺少GCC的电脑不影响Python上位机运行。

第三方内容见THIRD_PARTY_NOTICES.md，完整GPL见LICENSE。此包不提供满载性能保证。
