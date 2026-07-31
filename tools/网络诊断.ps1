# RSI 链路诊断 —— 判断 KRC 有没有把帧发到宿主
#
# 用批处理写过一版，放弃了：Windows 的 timeout.exe 会被 Git-Bash 的同名程序抢走，
# UTF-8 中文在 if(...) 块里会被代码页切碎，netstat 的中文标签还得按位置猜。
# PowerShell 能直接读 .NET 的 UDP 统计，与语言无关。

$ErrorActionPreference = 'Stop'
$HostIp   = '192.168.44.1'
$GuestIp  = '192.168.44.128'
$Port     = 59152
$CycleMs  = 12.0
$Expected = [int](1000.0 / $CycleMs)   # 每秒预期包数

function Get-UdpRx {
    [System.Net.NetworkInformation.IPGlobalProperties]::GetIPGlobalProperties().
        GetUdpIPv4Statistics().DatagramsReceived
}

Write-Host ''
Write-Host '  RSI 链路诊断' -ForegroundColor Cyan
Write-Host '  ============================================'
Write-Host ''
Write-Host '  先开着这个窗口，再去示教器启动 PoseTrack 程序。'
Write-Host '  最长采样 15 分钟，按 Ctrl+C 可随时结束。'
Write-Host ''

# ---- 静态检查 ----
Write-Host '  [1] 宿主是否在监听 ' -NoNewline
Write-Host "$HostIp`:$Port"
$ep = Get-NetUDPEndpoint -LocalPort $Port -ErrorAction SilentlyContinue
if (-not $ep) {
    Write-Host '      [X] 没有进程绑定该端口 —— rsi_host.exe 没运行，或没点"开始监听"' -ForegroundColor Red
} else {
    foreach ($e in $ep) {
        $p = Get-Process -Id $e.OwningProcess -ErrorAction SilentlyContinue
        Write-Host "      [OK] $($e.LocalAddress):$($e.LocalPort)  <- $($p.ProcessName) (PID $($e.OwningProcess))" -ForegroundColor Green
    }
}

Write-Host "  [2] 能否连通虚拟机 $GuestIp"
if (Test-Connection -ComputerName $GuestIp -Count 2 -Quiet -ErrorAction SilentlyContinue) {
    Write-Host '      [OK] 可达' -ForegroundColor Green
} else {
    Write-Host '      [X] ping 不通 —— 虚拟机没开，或网卡不在 VMnet1' -ForegroundColor Red
}

Write-Host '  [3] 防火墙是否放行'
$rules = Get-NetFirewallRule -Direction Inbound -Enabled True -Action Allow -ErrorAction SilentlyContinue |
         Where-Object { $_.DisplayName -like '*rsi_host*' }
if ($rules) {
    $profiles = ($rules | ForEach-Object { $_.Profile.ToString() } | Sort-Object -Unique) -join ', '
    Write-Host "      [OK] 有 rsi_host 入站放行规则，适用配置文件: $profiles" -ForegroundColor Green
    $cat = (Get-NetConnectionProfile -ErrorAction SilentlyContinue |
            ForEach-Object { $_.NetworkCategory.ToString() } | Sort-Object -Unique) -join ', '
    Write-Host "      当前网络类别: $cat"
} else {
    Write-Host '      [!] 没找到 rsi_host 的入站规则；若采样为零可先临时关防火墙验证' -ForegroundColor Yellow
}

Write-Host ''
Write-Host "  [4] 开始采样。$CycleMs ms 周期下 KRC 每秒约发 $Expected 个包。"
Write-Host '      现在去示教器启动程序……'
Write-Host ''

$prev = Get-UdpRx
$peak = 0
$hits = 0
$sw   = [Diagnostics.Stopwatch]::StartNew()

for ($i = 1; $i -le 900; $i++) {
    Start-Sleep -Seconds 1
    $cur = Get-UdpRx
    $d   = $cur - $prev
    $prev = $cur
    if ($d -gt 20) {
        $hits++
        if ($d -gt $peak) { $peak = $d }
        $t = $sw.Elapsed.ToString('mm\:ss')
        Write-Host ("      [{0}] 整机 UDP {1} 个/秒" -f $t, $d) -ForegroundColor Green
    }
}

Write-Host ''
Write-Host '  ============================================'
if ($peak -gt 20) {
    Write-Host "  采样期间有 UDP 流量（峰值 $peak 个/秒，共 $hits 秒）。" -ForegroundColor Yellow
    Write-Host ''
    Write-Host '  注意：这个计数器统计的是整机所有 UDP 包，不区分端口。' -ForegroundColor Yellow
    Write-Host '  浏览器、mDNS、VMware 自己的流量都会计入，所以"有流量"'
    Write-Host '  并不等于"KRC 在发帧"。判断依据以 rsi_host 界面为准：'
    Write-Host ''
    Write-Host '    状态栏 ● 已连接        -> KRC 确实在发，链路通了'
    Write-Host '    状态栏 ◐ 监听中(等待)  -> 一帧都没收到，上面的流量是别的'
    Write-Host ''
    Write-Host "  参考：12 ms 周期下 RSI 约 $Expected 个/秒。远高于此的峰值多半不是 RSI。"
} else {
    Write-Host '  采样期间整机 UDP 流量都几乎为零。' -ForegroundColor Yellow
    Write-Host ''
    Write-Host '  这个方向的结论是可靠的：连别的流量都没有，RSI 帧更不可能到。'
    Write-Host '  如果这期间程序确实在跑，说明 Ethernet 对象没能发出数据。查：'
    Write-Host '    1. KRC 网络配置里的地址是否与虚拟机网卡实际地址一致'
    Write-Host '    2. 虚拟机内的防火墙是否挡了出站 UDP'
    Write-Host '    3. 示教器上 RSIBad 之前的那条报错'
}
Write-Host ''
Read-Host '  按回车关闭'
