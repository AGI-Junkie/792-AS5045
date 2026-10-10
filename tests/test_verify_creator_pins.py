"""Run the real pin gate, including rejected netlist/configuration mutations.

python tests/test_verify_creator_pins.py [--software-project PATH]
Requires Creator-generated files; never writes the source project.
"""
import argparse
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
FILES = ("main.c", "codegentemp/DRV8825.v",
         "Generated_Source/PSoC5/cyfitter.h", "Generated_Source/PSoC5/HostUart.h")
SHELL = shutil.which("pwsh") or shutil.which("powershell")


def check(project, flags=(), expected_error=None):
    result = subprocess.run(
        [SHELL, "-NoProfile", "-ExecutionPolicy", "Bypass", "-File",
         str(ROOT / "verify-creator-pins.ps1"), "-ProjectPath", str(project), *flags],
        capture_output=True, text=True, errors="replace")
    output = result.stdout + result.stderr
    if expected_error is None:
        assert result.returncode == 0, output
        assert "PASS: 25 physical pins" in output, output
    else:
        assert result.returncode != 0 and expected_error in output, output


def replace(text, pattern, replacement):
    changed, count = re.subn(pattern, replacement, text, count=1, flags=re.S)
    assert count == 1, pattern
    return changed


def port(text, instance, terminal):
    return re.search(r"\b" + instance + r"\s*\(.*?\." + terminal +
                     r"\(\{?(\w+)(?:\[0:0\])?\}?\)", text, re.S).group(1)


def fault_edge(text):
    # USB D+ also has a falling-edge mode; mutate the FAULT instance's parameters.
    pattern = r"\bcy_psoc3_pins_\w+\s*#\((?P<params>(?:(?!\);).)*?)\)\s*FAULT\s*\("
    match = re.search(pattern, text, re.S)
    assert match, "FAULT pin parameters"
    params = replace(match['params'], r"\.intr_mode\(2'b10\)", ".intr_mode(2'b00)")
    return text[:match.start('params')] + params + text[match.end('params'):]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--software-project", type=Path)
    args = parser.parse_args()
    assert SHELL, "PowerShell is required"
    source = ROOT / "DRV8825.cydsn"
    originals = {name: (source / name).read_text() for name in FILES}
    assert "#define HW_TXDMA" in originals["main.c"]
    # Exercise DMA gates even when production temporarily uses CPU UART output.
    originals["main.c"] = replace(originals["main.c"], r"(#define\s+HW_TXDMA\s+)\([01]u\)", r"\g<1>(1u)")
    v = originals["codegentemp/DRV8825.v"]
    # Each corruption must fail for its intended reason, not just any error.
    mutations = [
        ("physical mapping", FILES[2],
         lambda s: replace(s, r"(#define\s+STEP1__0__SHIFT\s+)\d+u", r"\g<1>7u"), "STEP1 SHIFT"),
        ("SSI clock", FILES[1],
         lambda s: replace(s, r"(\bSsi\s*\(.*?\.sclk\()\w+(\))", r"\g<1>BrokenClock\2"), "Ssi.sclk"),
        ("SSI input", FILES[1],
         lambda s: replace(s, r"(\bSsi\s*\(.*?\.miso\()\w+(\))", r"\g<1>BrokenInput\2"), "Ssi.miso"),
        ("STEP polarity", FILES[1],
         lambda s: replace(s, r"(assign\s+" + port(v, "STEP2", "y") + r"\s*=\s*)~", r"\g<1>"), "STEP2 inversion"),
        ("fault edge", FILES[1],
         fault_edge, "FAULT.intr_mode"),
        ("fault ISR", FILES[1],
         lambda s: replace(s, r"(\bFaultIsr\s*\(.*?\.int_signal\()\w+(\))", r"\g<1>BrokenIrq\2"), "FaultIsr.int_signal"),
        ("DMA buffer", FILES[3],
         lambda s: replace(s, r"(#define\s+HostUart_TX_BUFFER_SIZE\s+\()4u", r"\g<1>256u"), "TX_BUFFER_SIZE"),
        ("DMA request", FILES[1],
         lambda s: replace(s, r"(\bTxDma\s*\(.*?\.drq\()\w+(\))", r"\g<1>BrokenDrq\2"), "TxDma.drq"),
        ("DMA request mask", FILES[3],
         lambda s: replace(s, r"\(0 << HostUart_TX_STS_FIFO_EMPTY_SHIFT\)",
                           "(1 << HostUart_TX_STS_FIFO_EMPTY_SHIFT)"), "UART DMA request mask"),
        ("missing UART", FILES[1],
         lambda s: replace(s, r"\bHostUart\s*\(", "MissingUart("), "Missing hardware instance: HostUart"),
        ("DMA sync clock", FILES[1],
         lambda s: replace(s, r"(\bTxReqSync\s*\(.*?\.clock\()\w+(\))", r"\g<1>1'b0\2"), "TxReqSync.clock"),
        ("DMA request type", FILES[1],
         lambda s: replace(s, r"\.drq_type\(2'b01\)", ".drq_type(2'b00)"), "TxDma.drq_type"),
    ]
    with tempfile.TemporaryDirectory(prefix="pin-gate-") as directory:
        project = Path(directory)
        for name, content in originals.items():
            target = project / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(content)
        check(project)
        check(project, ("-NamedInstances", "-HardwareUart"))
        (project / "main.c").write_text(replace(originals["main.c"], r"(#define\s+HW_TXDMA\s+)\(1u\)", r"\g<1>(0u)"))
        check(project, ("-NamedInstances", "-HardwareUart"))
        print("PASS: CPU UART with 4-byte FIFO")
        (project / FILES[3]).write_text(replace(originals[FILES[3]], r"(#define\s+HostUart_TX_BUFFER_SIZE\s+\()4u", r"\g<1>256u"))
        check(project, expected_error="TX_INTERRUPT_ENABLED")
        print("PASS: CPU UART software buffer without TX interrupt rejected")
        (project / FILES[3]).write_text(originals[FILES[3]])
        (project / "main.c").write_text(originals["main.c"])
        for label, name, mutate, error in mutations:
            (project / name).write_text(mutate(originals[name]))
            check(project, expected_error=error)
            (project / name).write_text(originals[name])
            print("PASS:", label, "rejected")
        pairs = re.findall(r"\n(\w+),(Pin_\d+),", (ROOT / "verify-creator-pins.ps1").read_text())
        assert len(pairs) == 23
        for name in (FILES[1], FILES[2]):
            content = originals[name]
            for functional, legacy in pairs:
                functional = "AS5045_DO" if functional == "DO" else functional
                content = re.sub(r"\b" + functional + r"(?=\b|__)", legacy, content)
            (project / name).write_text(content)
        check(project)
        check(project, ("-NamedInstances",), "Functional names incomplete")
        print("PASS: legacy names accepted by default, rejected in strict mode")
    if args.software_project:
        check(args.software_project)
        check(args.software_project, ("-NamedInstances", "-HardwareUart"))
        print("PASS: software-driven project compatibility")
    print(f"PASS: hardware pin gate and {len(mutations)} rejection checks")


if __name__ == "__main__":
    main()
