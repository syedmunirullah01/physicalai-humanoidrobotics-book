#!/usr/bin/env python3
"""
Readability Validation Script for Physical AI & Humanoid Robotics Textbook
Version: 1.0.0
Purpose: Validate content meets Flesch-Kincaid grade level 13-15 and Flesch Reading Ease 30-50

Usage:
    python scripts/validate-readability.py docs/preface.mdx
    python scripts/validate-readability.py docs/**/*.mdx --verbose
"""

import argparse
import re
import sys
from pathlib import Path
from typing import Dict, List, Tuple

try:
    import textstat
except ImportError:
    print("ERROR: textstat not installed. Run: pip install textstat")
    sys.exit(1)


def extract_text_from_mdx(file_path: Path) -> str:
    """
    Extract text content from MDX file, removing frontmatter, code blocks, and JSX.

    Args:
        file_path: Path to MDX file

    Returns:
        Plain text content for readability analysis
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Remove frontmatter (YAML between --- delimiters)
    content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL | re.MULTILINE)

    # Remove code blocks (```...```)
    content = re.sub(r'```[\s\S]*?```', '', content)

    # Remove inline code (`...`)
    content = re.sub(r'`[^`]+`', '', content)

    # Remove JSX components (e.g., <ComponentName>...</ComponentName>)
    content = re.sub(r'<[A-Z][^>]*>.*?</[A-Z][^>]*>', '', content, flags=re.DOTALL)

    # Remove self-closing JSX (e.g., <Component />)
    content = re.sub(r'<[A-Z][^>]*/>', '', content)

    # Remove HTML comments
    content = re.sub(r'<!--[\s\S]*?-->', '', content)

    # Remove Markdown links but keep text: [text](url) -> text
    content = re.sub(r'\[([^\]]+)\]\([^\)]+\)', r'\1', content)

    # Remove Markdown headings markers (##, ###, etc.) but keep text
    content = re.sub(r'^#{1,6}\s+', '', content, flags=re.MULTILINE)

    # Remove placeholder content markers
    content = re.sub(r'\[Content to be written:.*?\]', '', content)

    return content.strip()


def calculate_readability(text: str) -> Dict[str, float]:
    """
    Calculate readability metrics using textstat.

    Args:
        text: Plain text content

    Returns:
        Dictionary with readability scores
    """
    return {
        'flesch_kincaid_grade': textstat.flesch_kincaid_grade(text),
        'flesch_reading_ease': textstat.flesch_reading_ease(text),
        'gunning_fog': textstat.gunning_fog(text),
        'smog_index': textstat.smog_index(text),
        'automated_readability_index': textstat.automated_readability_index(text),
        'coleman_liau_index': textstat.coleman_liau_index(text),
        'word_count': textstat.lexicon_count(text, removepunct=True),
        'sentence_count': textstat.sentence_count(text),
        'avg_sentence_length': textstat.avg_sentence_length(text),
        'avg_syllables_per_word': textstat.avg_syllables_per_word(text)
    }


def validate_targets(metrics: Dict[str, float]) -> Tuple[bool, List[str]]:
    """
    Validate readability metrics against target thresholds.

    Targets from spec.md:
    - Flesch-Kincaid Grade Level: 13-15 (university undergraduate)
    - Flesch Reading Ease: 30-50 (Difficult/College level)

    Args:
        metrics: Readability metrics dictionary

    Returns:
        (passed, warnings) tuple where passed is True if all targets met
    """
    passed = True
    warnings = []

    fk_grade = metrics['flesch_kincaid_grade']
    fre_score = metrics['flesch_reading_ease']

    # Flesch-Kincaid Grade Level: Target 13-15, max acceptable 17
    if fk_grade < 13:
        warnings.append(f"⚠️  FK Grade {fk_grade:.1f} < 13 (too easy, may lack technical depth)")
        passed = False
    elif fk_grade > 17:
        warnings.append(f"❌ FK Grade {fk_grade:.1f} > 17 (too complex, exceeds graduate level)")
        passed = False
    elif fk_grade > 15:
        warnings.append(f"⚠️  FK Grade {fk_grade:.1f} > 15 (acceptable for graduate students, but target is 13-15)")

    # Flesch Reading Ease: Target 30-50, min acceptable 25
    if fre_score < 25:
        warnings.append(f"❌ FRE {fre_score:.1f} < 25 (incomprehensible, violates SC-009 zero jargon confusion)")
        passed = False
    elif fre_score < 30:
        warnings.append(f"⚠️  FRE {fre_score:.1f} < 30 (very difficult, target is 30-50)")
    elif fre_score > 60:
        warnings.append(f"⚠️  FRE {fre_score:.1f} > 60 (too easy, may lack rigor)")
        passed = False

    return passed, warnings


def format_report(file_path: Path, metrics: Dict[str, float], passed: bool, warnings: List[str], verbose: bool = False) -> str:
    """
    Format readability validation report.

    Args:
        file_path: Path to analyzed file
        metrics: Readability metrics
        passed: Whether validation passed
        warnings: List of warning messages
        verbose: Show detailed metrics

    Returns:
        Formatted report string
    """
    status_icon = "✅" if passed else "❌"
    report_lines = [
        f"\n{status_icon} {file_path}",
        f"   Flesch-Kincaid Grade: {metrics['flesch_kincaid_grade']:.1f} (target: 13-15)",
        f"   Flesch Reading Ease: {metrics['flesch_reading_ease']:.1f} (target: 30-50)"
    ]

    if warnings:
        report_lines.append("\n   Warnings:")
        for warning in warnings:
            report_lines.append(f"      {warning}")

    if verbose:
        report_lines.extend([
            f"\n   Detailed Metrics:",
            f"      Word Count: {metrics['word_count']}",
            f"      Sentence Count: {metrics['sentence_count']}",
            f"      Avg Sentence Length: {metrics['avg_sentence_length']:.1f} words",
            f"      Avg Syllables/Word: {metrics['avg_syllables_per_word']:.2f}",
            f"      Gunning Fog: {metrics['gunning_fog']:.1f}",
            f"      SMOG Index: {metrics['smog_index']:.1f}",
            f"      ARI: {metrics['automated_readability_index']:.1f}",
            f"      Coleman-Liau: {metrics['coleman_liau_index']:.1f}"
        ])

    return "\n".join(report_lines)


def main():
    parser = argparse.ArgumentParser(
        description="Validate readability of textbook content (Flesch-Kincaid 13-15, FRE 30-50)"
    )
    parser.add_argument(
        "files",
        nargs="+",
        type=Path,
        help="MDX file(s) to validate"
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Show detailed readability metrics"
    )
    parser.add_argument(
        "--fail-on-warning",
        action="store_true",
        help="Exit with error code if any warnings (not just failures)"
    )

    args = parser.parse_args()

    all_passed = True
    has_warnings = False

    for file_path in args.files:
        if not file_path.exists():
            print(f"❌ File not found: {file_path}")
            all_passed = False
            continue

        if not file_path.suffix in ['.md', '.mdx']:
            print(f"⚠️  Skipping non-Markdown file: {file_path}")
            continue

        # Extract text and calculate metrics
        text = extract_text_from_mdx(file_path)

        if not text or len(text.strip()) < 100:
            print(f"⚠️  {file_path}: Insufficient content for analysis (< 100 chars)")
            has_warnings = True
            continue

        metrics = calculate_readability(text)
        passed, warnings = validate_targets(metrics)

        # Print report
        print(format_report(file_path, metrics, passed, warnings, args.verbose))

        if not passed:
            all_passed = False
        if warnings:
            has_warnings = True

    # Summary
    print("\n" + "="*60)
    if all_passed:
        print("✅ All files passed readability validation")
        exit_code = 0
    else:
        print("❌ Some files failed readability validation")
        exit_code = 1

    if has_warnings and args.fail_on_warning:
        print("⚠️  Warnings detected (--fail-on-warning enabled)")
        exit_code = 1

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
