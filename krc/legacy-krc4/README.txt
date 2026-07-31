这两个文件是旧格式（.rsi + .rsi.xml），给 KR C4 / RSI 3.x 用的。

本机的 OfficeLite 是 RSI 5.0+，运行时只认 .rsix —— RSI_CREATE 会自动补
.rsix 扩展名，所以传 "PoseTrack.rsi" 会让它去找 PoseTrack.rsi.rsix 并报
"未找到文件"。实际部署请用上一级的 PoseTrack.rsix。

留着它们是因为 RSIVisual 能打开旧格式（已验证），换一台 KR C4 控制器时
可以直接用。
