# Capture a window by process/title into a PNG using PrintWindow.
# Does not require the window to be foreground.
# Usage: powershell -File tools/snap.ps1 -Title rsi_host -Out shot.png
param(
    [string]$Title = "rsi_host",
    [string]$Out   = "shot.png"
)

# VS2017 leaves LIB/INCLUDE pointing at paths that no longer exist, which makes
# Add-Type's inline C# compile fail. Only P/Invoke decls are needed here.
foreach ($v in 'LIB','INCLUDE','LIBPATH') {
    if (Test-Path "env:$v") { Remove-Item "env:$v" }
}

Add-Type -AssemblyName System.Drawing
Add-Type @"
using System;
using System.Drawing;
using System.Runtime.InteropServices;
public class Win {
    [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
    [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr h, IntPtr hdc, uint flags);
    [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int c);
    [StructLayout(LayoutKind.Sequential)] public struct RECT { public int L, T, R, B; }

    // PW_RENDERFULLCONTENT (2) is required for composited / GPU-backed content.
    public static Bitmap Grab(IntPtr h) {
        RECT r;
        if (!GetWindowRect(h, out r)) return null;
        int w = r.R - r.L, hh = r.B - r.T;
        if (w <= 0 || hh <= 0) return null;
        Bitmap bmp = new Bitmap(w, hh);
        using (Graphics g = Graphics.FromImage(bmp)) {
            IntPtr hdc = g.GetHdc();
            PrintWindow(h, hdc, 2);
            g.ReleaseHdc(hdc);
        }
        return bmp;
    }
}
"@ -ReferencedAssemblies System.Drawing

$proc = Get-Process | Where-Object {
    $_.MainWindowHandle -ne 0 -and
    ($_.ProcessName -like "*$Title*" -or $_.MainWindowTitle -like "*$Title*")
} | Select-Object -First 1
if (-not $proc) { Write-Error "no window matching '$Title'"; exit 1 }

$h = $proc.MainWindowHandle
[void][Win]::ShowWindow($h, 9)
Start-Sleep -Milliseconds 500

$bmp = [Win]::Grab($h)
if (-not $bmp) { Write-Error "grab failed"; exit 1 }
$bmp.Save($Out, [System.Drawing.Imaging.ImageFormat]::Png)
$w = $bmp.Width; $hh = $bmp.Height
$bmp.Dispose()
Write-Output "saved $Out ${w}x${hh}"
