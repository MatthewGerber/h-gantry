import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline

# Load your newly generated file
with open('cursive_alphabet_data.json', 'r') as f:
    alphabet = json.load(f)


def render_word(word):
    current_offset = 0.0
    strokes = []
    current_stroke = []

    for char in word:
        if char == ' ':
            current_offset += 0.5
            continue
        if char not in alphabet:
            continue

        # Offset character coordinates horizontally
        char_data = alphabet[char]
        for x, y, pen_up in char_data["points"]:
            current_stroke.append((x + current_offset, y))
            if pen_up == 1:
                strokes.append(current_stroke)
                current_stroke = []

        current_offset += char_data["advance_width"]

    if current_stroke:
        strokes.append(current_stroke)

    # Plot using cubic splines for ultimate smoothness
    plt.figure(figsize=(12, 3))
    for stroke in strokes:
        if len(stroke) < 2:
            continue
        x, y = zip(*stroke)
        if len(stroke) > 3:
            t = np.linspace(0, 1, len(stroke))
            t_new = np.linspace(0, 1, len(stroke) * 10)
            x = make_interp_spline(t, x, k=3)(t_new)
            y = make_interp_spline(t, y, k=3)(t_new)
        plt.plot(x, y, color='indigo', lw=3, solid_capstyle='round')

    plt.axis('off')
    plt.ylim(-0.8, 1.8)
    plt.show()


# Run the test phrase containing the complex multi-stroke letters
render_word("H")