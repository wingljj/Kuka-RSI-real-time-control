# Self-signed code signing: create (or reuse) the CN=wingljj certificate,
# then sign every file passed on the command line.
# Usage: powershell -ExecutionPolicy Bypass -File tools\sign_dist.ps1 a.exe b.exe
# NOTE: a self-signed cert cannot satisfy SmartScreen ("unknown publisher");
# a paid CA cert is required for that. Target machines may import the exported
# dist\wingljj-codesign.cer into "Trusted People"/"Trusted Root" so the
# signature verifies as valid.
# ASCII only: Windows PowerShell 5.1 reads BOM-less scripts as ANSI.
param([Parameter(ValueFromRemainingArguments = $true)][string[]]$Files)

$ErrorActionPreference = 'Stop'
$signtool = 'C:\Program Files (x86)\Windows Kits\10\bin\10.0.19041.0\x64\signtool.exe'

$cert = Get-ChildItem Cert:\CurrentUser\My -CodeSigningCert |
        Where-Object { $_.Subject -eq 'CN=wingljj' } |
        Sort-Object NotAfter -Descending | Select-Object -First 1
if (-not $cert) {
    $cert = New-SelfSignedCertificate -Type CodeSigningCert -Subject 'CN=wingljj' `
        -CertStoreLocation Cert:\CurrentUser\My -NotAfter (Get-Date).AddYears(5) `
        -KeyAlgorithm RSA -KeyLength 3072 -HashAlgorithm SHA256
    Write-Host "created new cert $($cert.Thumbprint)"
} else {
    Write-Host "reusing cert $($cert.Thumbprint)"
}

# Export the public certificate so target machines can trust it.
Export-Certificate -Cert $cert -FilePath "$PSScriptRoot\..\dist\wingljj-codesign.cer" | Out-Null

# Timestamp keeps signatures valid after cert expiry. Try several servers;
# fall back to an untimestamped signature if none is reachable.
$tsServers = @('http://timestamp.digicert.com',
               'http://timestamp.sectigo.com',
               'http://time.certum.pl')

foreach ($f in $Files) {
    $done = $false
    foreach ($ts in $tsServers) {
        & $signtool sign /sha1 $cert.Thumbprint /fd SHA256 /td SHA256 /tr $ts $f 2>$null
        if ($LASTEXITCODE -eq 0) { $done = $true; Write-Host "signed+ts($ts): $f"; break }
    }
    if (-not $done) {
        & $signtool sign /sha1 $cert.Thumbprint /fd SHA256 $f
        if ($LASTEXITCODE -ne 0) { throw "signing failed: $f" }
        Write-Host "signed(no timestamp): $f"
    }
}
