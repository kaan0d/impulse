// Blocks a commit whose types or unit tests are broken. The browser tests are slower and run in CI (npm run e2e).
// Plain Node on purpose: npm's own launcher on Windows is a bash script, and IDEs and Git GUIs often have no bash.
// `shell: true` lets the OS find npm (npm.cmd on Windows). Skip once with: git commit --no-verify
import { spawnSync } from 'node:child_process';

for (const script of ['typecheck', 'test']) {
  const { status } = spawnSync('npm', ['run', script], { stdio: 'inherit', shell: true });
  if (status !== 0) process.exit(status ?? 1);
}
