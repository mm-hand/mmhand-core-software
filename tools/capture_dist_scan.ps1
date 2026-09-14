param(
    [string]$Label = "dist",
    [string]$PortName = "COM11",
    [int]$Seconds = 12,
    [int]$DelaySeconds = 5,
    [int]$BaudRate = 921600
)

$port = New-Object System.IO.Ports.SerialPort $PortName, $BaudRate, 'None', 8, 'One'
$port.ReadTimeout = 500
$records = @{}
$samples = 0

try {
    $port.Open()
    $port.DiscardInBuffer()
    if ($DelaySeconds -gt 0) {
        Start-Sleep -Seconds $DelaySeconds
    }

    $start = Get-Date
    while (((Get-Date) - $start).TotalSeconds -lt $Seconds) {
        try {
            $line = $port.ReadLine()
        } catch [System.TimeoutException] {
            continue
        }

        if ($line -match 'OK\s+pat=([01]{5})\s+DistMag=\s*(\d+)') {
            $pat = $Matches[1]
            $mag = [int]$Matches[2]
            if (-not $records.ContainsKey($pat) -or $mag -gt $records[$pat].mag) {
                $records[$pat] = [pscustomobject]@{
                    pat = $pat
                    mag = $mag
                }
            }
            $samples++
        }
    }
} finally {
    if ($port.IsOpen) {
        $port.Close()
    }
}

Write-Output "label=$Label samples=$samples patterns=$($records.Count)"
$records.Values |
    Sort-Object mag -Descending |
    Select-Object -First 12 |
    ForEach-Object {
        Write-Output ("TOP pat={0} distMag={1}" -f $_.pat, $_.mag)
    }
