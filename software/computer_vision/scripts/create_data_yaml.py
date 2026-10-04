from pathlib import Path
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]

CLASSES_FILE = PROJECT_ROOT / "custom_data" / "classes.txt"
DATA_DIR = PROJECT_ROOT / "data"
OUTPUT_FILE = PROJECT_ROOT / "data.yaml"


def main():
    if not CLASSES_FILE.exists():
        raise FileNotFoundError(
            f"Classes file not found: {CLASSES_FILE}"
        )

    classes = []

    with CLASSES_FILE.open("r") as file:
        for line in file:
            class_name = line.strip()

            if class_name:
                classes.append(class_name)

    config = {
        "path": str(DATA_DIR),
        "train": "train/images",
        "val": "validation/images",
        "nc": len(classes),
        "names": classes,
    }

    with OUTPUT_FILE.open("w") as file:
        yaml.safe_dump(
            config,
            file,
            sort_keys=False,
        )

    print(f"Created: {OUTPUT_FILE}")
    print()
    print("Classes:")

    for index, class_name in enumerate(classes):
        print(f"  {index}: {class_name}")


if __name__ == "__main__":
    main()