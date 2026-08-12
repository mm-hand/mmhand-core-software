param(
    [string]$Label = "scan",
    [string]$PortName = "COM11",
    [int]$Seconds = 10,
    [int]$DelaySeconds = 0,
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

        if ($line -match 'OK\s+pat=([01]{5})\s+Fx=\s*(-?\d+)\s+Fy=\s*(-?\d+)\s+Fz=\s*(\d+)') {
            $pat = $Matches[1]
            $fx = [int]$Matches[2]
            $fy = [int]$Matches[3]
            $fz = [int]$Matches[4]
            $mag = [Math]::Max([Math]::Max([Math]::Abs($fx), [Math]::Abs($fy)), $fz)
            if (-not $records.ContainsKey($pat) -or $mag -gt $records[$pat].mag) {
                $records[$pat] = [pscustomobject]@{
                    pat = $pat
                    fx = $fx
                    fy = $fy
                    fz = $fz
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
        Write-Output ("TOP pat={0} mag={1} fx={2} fy={3} fz={4}" -f $_.pat, $_.mag, $_.fx, $_.fy, $_.fz)
    }
