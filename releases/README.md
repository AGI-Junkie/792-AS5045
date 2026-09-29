# releases/

Only firmware builds that have actually run on the machine live here.

- One folder per build, named after the `fw=` id the firmware prints in its LINK line.
- Add a build only after a host log shows that `fw=` id running.
- Tag it `hw-tested/<fw id>`. Identity is the id string plus the hex sha256; nothing proves the
  bytes flashed from Creator equal the archived hex.
- `.gitignore` ignores `*.hex` everywhere except here.
