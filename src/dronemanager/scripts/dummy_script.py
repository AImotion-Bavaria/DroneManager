"""Dummy script for testing."""
import sys


def success() -> int:
    """Dummy function with zero return code.

    Returns:
        0.
    """
    return 0


def fail() -> int:
    """Dummy function with non-zero return code.

    Returns:
        1.
    """
    return 1


def main():
    """Dummy main function.

    Calls sys.exit with different codes based on script arguments. Also prints to either sys.stdout or sys.stderr
    based on arguments.
    """
    if len(sys.argv) > 1:
        if sys.argv[1] == "--success":
            print("Success!")
            code = success()
        elif sys.argv[1] == "--fail":
            sys.stderr.write("Failure!")
            code = fail()
        else:
            code = 0
    else:
        print("No arguments")
        code = 0
    exit(code)


if __name__ == "__main__":
    main()
