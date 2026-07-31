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
        Write-Host ("      [{0}] 收到 {1} 个包   <== KRC 正在发帧" -f $t, $d) -ForegroundColor Green
    }
}

Write-Host ''
Write-Host '  ============================================'
if ($peak -gt 20) {
    Write-Host "  KRC 确实在发帧（峰值 $peak 个/秒，共 $hits 秒有流量）。" -ForegroundColor Green
    Write-Host ''
    Write-Host '  问题在应答方向，按这个顺序查：'
    Write-Host '    1. rsi_host 界面的"不匹配"计数 —— 非零说明 IPOC 回显有问题'
    Write-Host '    2. SENTYPE：ethernet.xml 写 ImFree，配置里 sen_type 也必须是 ImFree'
    Write-Host "    3. 峰值 $peak 对应实测周期约 $([math]::Round(1000.0/$peak,1)) ms"
    Write-Host '       若不是 12 ms，rsi_config.json 的 cycle_ms 必须改，否则每个增量都错倍数'
} else {
    Write-Host '  整个采样期间没有成片的包到达宿主。' -ForegroundColor Yellow
    Write-Host ''
    Write-Host '  如果这期间程序确实在跑，说明 Ethernet 对象没能发出数据。查：'
    Write-Host '    1. OfficeLite 里 RSI 绑的网卡是不是 192.168.44.x 那块'
    Write-Host '    2. 虚拟机内的防火墙是否挡了出站 UDP'
    Write-Host '    3. 示教器上 RSIBad 之前的那条报错 —— 首个报错信息量最大'
}
Write-Host ''
Read-Host '  按回车关闭'
