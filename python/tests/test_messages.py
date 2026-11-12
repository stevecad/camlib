"""Messages are written as text: a number is separated from its words
("exceeds 64 KiB", "requires 1..16"), never run together ("exceeds64KiB")."""

from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[2]
SOURCES = ("src", "bindings/python/src", "python/camlib", "examples")
WORDS = "exceeds|exceed|requires|require|least|needs|need|most|limit|is|be|and|or|below|integer|version|than|within|to|of|are|cap"
RUN_TOGETHER = re.compile(rf"\b({WORDS})\d|\d(KiB|MiB|bytes|segments|events)\b")
LITERAL = re.compile(r'"(?:[^"\\\n]|\\.)*"' + "|" + r"'(?:[^'\\\n]|\\.)*'")


class MessageSpacingTest(unittest.TestCase):
    def test_numbers_are_separated_from_words_in_messages(self):
        found = []
        for base in SOURCES:
            for path in sorted((ROOT / base).rglob("*")):
                if path.suffix not in (".rs", ".py"):
                    continue
                for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
                    # Rust lifetimes ('a) are not string literals.
                    literals = (
                        LITERAL.finditer(line) if path.suffix == ".py" else re.finditer(r'"(?:[^"\\]|\\.)*"', line)
                    )
                    if any(RUN_TOGETHER.search(m.group(0)) for m in literals):
                        found.append(f"{path.relative_to(ROOT)}:{number}: {line.strip()}")
        self.assertEqual(found, [])


if __name__ == "__main__":
    unittest.main()
