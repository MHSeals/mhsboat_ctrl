import os


# TODO: don't hard code the paths lol
def main():
    v9 = "https://github.com/MHSeals/buoy-model/releases/download/V9/best.pt"
    v10 = "https://github.com/MHSeals/buoy-model/releases/download/V10/best.pt"
    os.system(f"wget '{v9}' -O /root/roboboat_ws/src/mhsboat_ctrl/mhsboat_ctrl/v9.pt")
    os.system(f"wget '{v10}' -O /root/roboboat_ws/src/mhsboat_ctrl/mhsboat_ctrl/v10.pt")


if __name__ == "__main__":
    main()
