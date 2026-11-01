Option Explicit
Dim files, shell, folder, pythonPath, scriptPath, command
Set files = CreateObject("Scripting.FileSystemObject")
Set shell = CreateObject("WScript.Shell")
folder = files.GetParentFolderName(WScript.ScriptFullName)
pythonPath = shell.Environment("PROCESS")("PAI_MONITOR_PYTHON")
If Len(pythonPath) = 0 Then pythonPath = "pythonw.exe"
scriptPath = files.BuildPath(folder, "native_monitor.py")
command = Chr(34) & pythonPath & Chr(34) & " " & Chr(34) & scriptPath & Chr(34)
On Error Resume Next
shell.Run command, 0, False
If Err.Number <> 0 Then
    MsgBox "Python with Tk was not found. Set PAI_MONITOR_PYTHON. See docs/SETUP.md.", 16, "Physical AI Monitor"
    WScript.Quit 1
End If
