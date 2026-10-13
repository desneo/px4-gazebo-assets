"""离线验证单入口保留验收退出码并清理本次启动的进程。"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest


class LauncherTests(unittest.TestCase):
    def test_exit_status_and_cleanup(self):
        for code in (0, 2):
            with self.subTest(code=code), tempfile.TemporaryDirectory(prefix='rocket-launch-test-') as directory:
                root = Path(directory)
                shutil.copy(Path(__file__).with_name('run_check.sh'), root / 'run_check.sh')
                run = root / 'runtime'
                run.mkdir()
                (root / 'run.sh').write_text('''#!/bin/sh
set -eu
mkdir -p "$FAKE_RUN/models/rocket_racing_quad"
touch "$FAKE_RUN/px4.log" "$FAKE_RUN/alive"
trap 'rm -f "$FAKE_RUN/alive"; exit 0' TERM INT EXIT
printf '运行目录：%s\\nGazebo 分区：test_partition\\n' "$FAKE_RUN"
while :; do sleep .1; done
''')
                bindir = root / 'bin'; bindir.mkdir()
                fake_python = bindir / 'python3'
                fake_python.write_text('''#!/bin/sh
[ "$GZ_PARTITION" = test_partition ] || exit 99
case "$*" in *--ground-truth*) ;; *) exit 98 ;; esac
exit "$FAKE_CODE"
''')
                fake_python.chmod(0o755)
                env = dict(os.environ, PATH=str(bindir) + os.pathsep + os.environ['PATH'],
                           FAKE_RUN=str(run), FAKE_CODE=str(code), TMPDIR=str(root))
                result = subprocess.run(['sh', str(root / 'run_check.sh'), 'headless', '/tmp/example.json'],
                    env=env, capture_output=True, text=True, timeout=10)
                self.assertEqual(result.returncode, code, result.stdout + result.stderr)
                self.assertFalse((run / 'alive').exists(), '本次启动进程未清理')


if __name__ == '__main__':
    unittest.main()
