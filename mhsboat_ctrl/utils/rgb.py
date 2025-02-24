class rgb:
    def __init__(self, r, g, b):
        self.r = r
        self.g = g
        self.b = b

    def __str__(self):
        return f"rgb({self.r}, {self.g}, {self.b})"

    def __repr__(self):
        return f"rgb({self.r}, {self.g}, {self.b})"

    def as_bgr(self) -> tuple:
        return (self.b, self.g, self.r)

    def invert(self):
        return rgb(255 - self.r, 255 - self.g, 255 - self.b)

    def text_color(self):
        luma = 0.2126 * self.r + 0.7152 * self.g + 0.0722 * self.b

        if luma < 40:
            return rgb(255, 255, 255)
        else:
            return rgb(0, 0, 0)
