Usage
-----

This tmuxinator config was generated from `.tmux/session.sh` to recreate the same tmux session named `quadruped_dev`.

Files created:
- `.tmuxinator/quadruped_dev.yml` — tmuxinator project file.

Quick start options:

1) Use the config directly from the repo (no install to user config dir required):

```bash
# from project root
tmuxinator start .tmuxinator/quadruped_dev.yml
```

2) or copy it to your user tmuxinator directory so you can start it by name:

```bash
mkdir -p ~/.tmuxinator
cp .tmuxinator/quadruped_dev.yml ~/.tmuxinator/quadruped_dev.yml
# then start it by name
tmuxinator start quadruped_dev
```

Notes and environment:
- Config uses absolute paths from the original script. If your paths differ, edit `.tmuxinator/quadruped_dev.yml` accordingly.
- tmuxinator requires $SHELL and $EDITOR in your environment. If you see a warning after install, set these in your shell config (e.g., `export EDITOR=vim` and `export SHELL=/bin/bash`).
- To verify installation: `tmuxinator version` and `tmux -V`.

If you want, I can copy the config into `~/.tmuxinator/` for you automatically (requires write permission to your home directory).