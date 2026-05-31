import subprocess


def test_urdf_validate_command_is_on_path():
    result = subprocess.run(
        ["urdf_validate", "--help"], capture_output=True, text=True
    )
    assert result.returncode == 0, (
        f"urdf_validate --help exited {result.returncode}\n"
        f"stdout: {result.stdout}\n"
        f"stderr: {result.stderr}"
    )


def test_urdf_validate_prints_something():
    result = subprocess.run(
        ["urdf_validate", "--help"], capture_output=True, text=True
    )
    assert result.stdout.strip() != ""
