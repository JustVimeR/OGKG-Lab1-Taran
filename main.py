import math
import numpy as np
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.widgets import Button
import random
import matplotlib
matplotlib.use('TkAgg')

plt.ion()
def compute_convex_hull(points):
    # Обчислення опуклої оболонки точок
    hull = ConvexHull(points)
    return hull


def compute_orientation(curr_point, next_point):
    # Обчислення орієнтації ребра за допомогою atan2
    angle = math.atan2(next_point[1] - curr_point[1], next_point[0] - curr_point[0])
    return angle


def rotate_points(points, angle):
    # Обертання точок за заданим кутом
    rotated_points = []
    for point in points:
        x = point[0] * math.cos(angle) - point[1] * math.sin(angle)
        y = point[0] * math.sin(angle) + point[1] * math.cos(angle)
        rotated_points.append((x, y))
    return rotated_points


def compute_smallest_surrounding_rectangle(points):
    # Обчислення опуклої оболонки точок
    hull = compute_convex_hull(points)

    # Ініціалізація змінних
    min_area = float('inf')
    min_angle = 0.0
    min_rect = None

    # Ітерація по ребрах опуклої оболонки
    for i in range(len(hull.vertices)):
        # Отримання індексів поточної та наступної вершин
        curr_index = hull.vertices[i]
        next_index = hull.vertices[(i + 1) % len(hull.vertices)]

        # Отримання координат поточних та наступних вершин
        curr_point = hull.points[curr_index]
        next_point = hull.points[next_index]

        # Обчислення орієнтації ребра
        angle = compute_orientation(curr_point, next_point)

        # Обертання опуклої оболонки з використанням орієнтації ребра
        rotated_hull = rotate_points(hull.points, angle)

        # Перетворення оберненої оболонки в масив NumPy
        rotated_hull = np.array(rotated_hull)

        # Обчислення площі обмежуючого прямокутника з мін./макс. x/y координат оберненої опуклої оболонки
        min_x = np.min(rotated_hull[:, 0])
        max_x = np.max(rotated_hull[:, 0])
        min_y = np.min(rotated_hull[:, 1])
        max_y = np.max(rotated_hull[:, 1])
        area = (max_x - min_x) * (max_y - min_y)

        # Перевірка, чи поточний прямокутник має мінімальну площу
        if area < min_area:
            min_area = area
            min_angle = angle
            min_rect = (min_x, min_y, max_x, max_y)

    return min_rect, min_angle


fig_points, ax_points = plt.subplots()
fig_result, ax_result = plt.subplots()
ax_points.set_xlim(0, 100)
ax_points.set_ylim(0, 100)


def plot_convex_hull(points):
    # Візуалізація опуклої оболонки
    hull = compute_convex_hull(points)
    ax_result.clear()
    for simplex in hull.simplices:
        ax_result.plot(points[simplex][:, 0], points[simplex][:, 1], 'k-')

    ax_result.scatter(points[:, 0], points[:, 1], color='b')
    ax_result.axis('equal')
    ax_result.set_xlabel('X')
    ax_result.set_ylabel('Y')
    ax_result.set_title('Опукла оболонка')
    fig_result.canvas.draw()


def plot_smallest_surrounding_rectangle(points, smallest_rect, orientation):
    # Візуалізація найменшого описуючого прямокутника
    rotated_points = rotate_points([smallest_rect[:2], (smallest_rect[2], smallest_rect[1]), smallest_rect[2:],
                                    (smallest_rect[0], smallest_rect[3])], -orientation)
    rotated_rect = Polygon(rotated_points, edgecolor='r', facecolor='none')

    ax_result.add_patch(rotated_rect)
    ax_result.scatter(points[:, 0], points[:, 1], color='b')
    ax_result.axis('equal')
    ax_result.set_xlabel('X')
    ax_result.set_ylabel('Y')
    ax_result.set_title('Найменший описуючий прямокутник')
    fig_result.canvas.draw()


def compute_and_plot(points):
    # Обчислення найменшого описуючого прямокутника і орієнтації
    smallest_rect, orientation = compute_smallest_surrounding_rectangle(points)
    print("Найменший описуючий прямокутник:", smallest_rect)
    print("Орієнтація:", math.degrees(orientation))

    # Візуалізація опуклої оболонки та найменшого описуючого прямокутника
    plot_convex_hull(points)
    plot_smallest_surrounding_rectangle(points, smallest_rect, orientation)


# Масив точок
points = []


def onclick(event):
    # Отримання координати точки, доданої мишкою
    x = event.xdata
    y = event.ydata
    if x is not None and y is not None:
        # Перевірка, чи клік не відбувається на кнопці
        if event.button == 1 and event.inaxes != compute_button_ax:
            points.append((x, y))
            ax_points.scatter(x, y, color='b')
            fig_points.canvas.draw()


def compute_button_clicked(event):
    compute_and_plot(np.array(points))


def generate_random_points():
    ax_points.clear()
    ax_points.set_xlim(0, 100)
    ax_points.set_ylim(0, 100)
    ax_result.clear()
    points.clear()

    # рандомні точки
    random_points = [(random.uniform(0, 100), random.uniform(0, 100)) for _ in range(100)]

    # рандомні точки в масив
    for point in random_points:
        points.append(point)
        ax_points.scatter(point[0], point[1], color='b')

    fig_points.canvas.draw()


def clear_points():
    ax_points.clear()
    ax_points.set_xlim(0, 100)
    ax_points.set_ylim(0, 100)
    ax_result.clear()
    points.clear()
    fig_points.canvas.draw()


# Налаштування кнопки "Очистити"
clear_button_ax = fig_points.add_axes([0.4, 0.92, 0.1, 0.075])
clear_button = Button(clear_button_ax, 'Очистити')
clear_button.on_clicked(lambda event: clear_points())

# Налаштування кнопки "Рандом"
random_button_ax = fig_points.add_axes([0.6, 0.92, 0.1, 0.075])
random_button = Button(random_button_ax, 'Random')
random_button.on_clicked(lambda event: generate_random_points())

# Налаштування кнопки "Обрахувати"
compute_button_ax = fig_points.add_axes([0.8, 0.92, 0.1, 0.075])
compute_button = Button(compute_button_ax, 'Обрахувати')
compute_button.on_clicked(compute_button_clicked)

# Налаштування події "Клік мишкою"
fig_points.canvas.mpl_connect('button_press_event', onclick)

plt.show()

plt.show(block=True)

