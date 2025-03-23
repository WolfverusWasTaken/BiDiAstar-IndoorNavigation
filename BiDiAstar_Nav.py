import pygame
import sys
import heapq

# ======== BIDIRECTIONAL ASTAR MODULE ========
def bidirectional_astar(grid, start, goal):
    # Detect waypoints dynamically
    waypoints = []
    lifts = {}
    levels = len(grid)

    for level in range(levels):
        for y in range(len(grid[level])):
            for x in range(len(grid[level][y])):
                if grid[level][y][x] == "E":
                    if level + 1 < levels and grid[level + 1][y][x] == "E":
                        waypoints.append((x, y, level, level + 1))
                elif grid[level][y][x] == "L":
                    if (x, y) not in lifts:
                        lifts[(x, y)] = []
                    lifts[(x, y)].append(level)

    for (x, y), lvls in lifts.items():
        for l1 in lvls:
            for l2 in lvls:
                if l1 != l2:
                    waypoints.append((x, y, l1, l2))

    class Node:
        def __init__(self, x, y, level, parent=None, g=0, h=0):
            self.x = x
            self.y = y
            self.level = level
            self.parent = parent
            self.g = g
            self.h = h
            self.f = g + h

        def __lt__(self, other):
            return self.f < other.f

    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])

    neighbor_cache = {}

    def get_neighbors(node):
        key = (node.x, node.y, node.level)
        if key in neighbor_cache:
            return neighbor_cache[key]

        neighbors = []
        x, y, level = node.x, node.y, node.level
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]

        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= ny < len(grid[level]) and 0 <= nx < len(grid[level][ny]):
                if grid[level][ny][nx] != "#":
                    neighbors.append((nx, ny, level, 1))

        for (wx, wy, from_lvl, to_lvl) in waypoints:
            if x == wx and y == wy and level == from_lvl:
                level_diff = abs(from_lvl - to_lvl)
                cost = 1
                if grid[from_lvl][wy][wx] == "E":
                    cost = 0.5 if level_diff == 1 else 2
                elif grid[from_lvl][wy][wx] == "L":
                    cost = 1
                neighbors.append((wx, wy, to_lvl, cost))

        neighbor_cache[key] = neighbors
        return neighbors

    def reconstruct_path(n1, n2):
        path1 = []
        path2 = []

        while n1:
            path1.append((n1.x, n1.y, n1.level))
            n1 = n1.parent
        while n2:
            path2.append((n2.x, n2.y, n2.level))
            n2 = n2.parent

        path1.reverse()
        path2.reverse()
        return path1 + path2[1:]

    open_start = []
    open_goal = []
    node_map_start = {}
    node_map_goal = {}
    closed_start = set()
    closed_goal = set()

    start_node = Node(*start, None, 0, heuristic(start, goal))
    goal_node = Node(*goal, None, 0, heuristic(goal, start))

    heapq.heappush(open_start, start_node)
    heapq.heappush(open_goal, goal_node)

    while open_start and open_goal:
        current_start = heapq.heappop(open_start)
        if (current_start.x, current_start.y, current_start.level) in closed_goal:
            meeting = current_start
            break

        closed_start.add((current_start.x, current_start.y, current_start.level))

        for nx, ny, nl, cost in get_neighbors(current_start):
            if (nx, ny, nl) in closed_start:
                continue
            g = current_start.g + cost
            node = Node(nx, ny, nl, current_start, g, heuristic((nx, ny, nl), goal))
            if (nx, ny, nl) not in node_map_start or g < node_map_start[(nx, ny, nl)].g:
                node_map_start[(nx, ny, nl)] = node
                heapq.heappush(open_start, node)

        current_goal = heapq.heappop(open_goal)
        if (current_goal.x, current_goal.y, current_goal.level) in closed_start:
            meeting = current_goal
            break

        closed_goal.add((current_goal.x, current_goal.y, current_goal.level))

        for nx, ny, nl, cost in get_neighbors(current_goal):
            if (nx, ny, nl) in closed_goal:
                continue
            g = current_goal.g + cost
            node = Node(nx, ny, nl, current_goal, g, heuristic((nx, ny, nl), start))
            if (nx, ny, nl) not in node_map_goal or g < node_map_goal[(nx, ny, nl)].g:
                node_map_goal[(nx, ny, nl)] = node
                heapq.heappush(open_goal, node)

    return reconstruct_path(
        node_map_start[(meeting.x, meeting.y, meeting.level)],
        node_map_goal[(meeting.x, meeting.y, meeting.level)]
    )


# ======== PYGAME SCRIPT STARTS HERE ========

GRID_WIDTH = 20
GRID_HEIGHT = 12
FLOORS = 3
TILE_SIZE = 15
FLOOR_GAP = 15
PADDING_LEFT = 250
WINDOW_WIDTH = PADDING_LEFT + GRID_WIDTH * TILE_SIZE
WINDOW_HEIGHT = FLOORS * (GRID_HEIGHT * TILE_SIZE + FLOOR_GAP) + 130

WHITE = (255, 255, 255)
GRAY = (210, 210, 210)
BLACK = (0, 0, 0)
GREEN = (50, 205, 50)
ORANGE = (255, 165, 0)
BLUE = (30, 144, 255)
RED = (255, 0, 0)

pygame.init()
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("3D Building Navigation - Bidirectional A*")
font = pygame.font.SysFont("arial", 12)

# Initialize grid
grid = [[[ "." for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)] for _ in range(FLOORS)]
door_labels = []
label_to_coord = {}
ROOMS_PER_FLOOR = 4

# Fill grid with rooms, doors, lifts, and escalators
for z in range(FLOORS):
    for room in range(ROOMS_PER_FLOOR):
        rx = room * (GRID_WIDTH // ROOMS_PER_FLOOR)
        ry = 2
        dx, dy = rx + 2, ry
        grid[z][dy][dx] = "D"
        label = f"Ex-{z+1:02}-{room+1:02}"
        door_labels.append(label)
        label_to_coord[label] = (dx, dy, z)

        for i in range(rx, rx + 5):
            if 0 <= i < GRID_WIDTH:
                grid[z][dy][i] = "#"
        grid[z][dy][dx] = "D"
        if dy - 1 >= 0:
            grid[z][dy - 1][dx] = "#"

    lift_x, lift_y = 1, GRID_HEIGHT // 2
    esc_x, esc_y = GRID_WIDTH - 2, GRID_HEIGHT // 2
    grid[z][lift_y][lift_x] = "L"
    grid[z][esc_y][esc_x] = "E"
    label_to_coord[f"lift{z+1:02}"] = (lift_x, lift_y, z)
    label_to_coord[f"esc{z+1:02}"] = (esc_x, esc_y, z)

dropdown_options = door_labels + [f"lift{z+1:02}" for z in range(FLOORS)] + [f"esc{z+1:02}" for z in range(FLOORS)]

selected_start = None
selected_goal = None
dropdown_open = None
dropdown_scroll = 0
max_dropdown_display = 10
path_result = []

def draw_grid():
    screen.fill(WHITE)

    for z in range(FLOORS):
        offset_y = z * (GRID_HEIGHT * TILE_SIZE + FLOOR_GAP) + 50
        screen.blit(font.render(f"Floor {z+1}", True, BLACK), (PADDING_LEFT, offset_y - 20))
        for y in range(GRID_HEIGHT):
            for x in range(GRID_WIDTH):
                rect = pygame.Rect(PADDING_LEFT + x * TILE_SIZE, offset_y + y * TILE_SIZE, TILE_SIZE, TILE_SIZE)
                pygame.draw.rect(screen, GRAY, rect, 1)
                val = grid[z][y][x]
                if val == "#":
                    pygame.draw.rect(screen, BLACK, rect)
                elif val == "D":
                    pygame.draw.rect(screen, GREEN, rect)
                    screen.blit(font.render("D", True, BLACK), (rect.x + 3, rect.y + 1))
                elif val == "L":
                    pygame.draw.rect(screen, BLUE, rect)
                    screen.blit(font.render("L", True, BLACK), (rect.x + 3, rect.y + 1))
                elif val == "E":
                    pygame.draw.rect(screen, ORANGE, rect)
                    screen.blit(font.render("E", True, BLACK), (rect.x + 3, rect.y + 1))

    for (x, y, z) in path_result:
        offset_y = z * (GRID_HEIGHT * TILE_SIZE + FLOOR_GAP) + 50
        cx = PADDING_LEFT + x * TILE_SIZE + TILE_SIZE // 2
        cy = offset_y + y * TILE_SIZE + TILE_SIZE // 2
        pygame.draw.circle(screen, RED, (cx, cy), 3)

    pygame.draw.rect(screen, GRAY, (10, 20, 230, 110), 0)
    pygame.draw.rect(screen, WHITE, (20, 30, 200, 30))
    pygame.draw.rect(screen, BLACK, (20, 30, 200, 30), 1)
    screen.blit(font.render(f"Start: {selected_start}", True, BLACK), (25, 35))
    pygame.draw.rect(screen, WHITE, (20, 70, 200, 30))
    pygame.draw.rect(screen, BLACK, (20, 70, 200, 30), 1)
    screen.blit(font.render(f"Goal: {selected_goal}", True, BLACK), (25, 75))

    if dropdown_open:
        visible_options = dropdown_options[dropdown_scroll:dropdown_scroll + max_dropdown_display]
        for i, label in enumerate(visible_options):
            rect = pygame.Rect(20, 120 + i * 25, 200, 25)
            pygame.draw.rect(screen, WHITE, rect)
            pygame.draw.rect(screen, BLACK, rect, 1)
            screen.blit(font.render(label, True, BLACK), (25, 125 + i * 25))

    y_offset = WINDOW_HEIGHT - 100
    screen.blit(font.render("Legend:", True, BLACK), (20, y_offset))
    legend = [
        ("Green", "D - Door"),
        ("Blue", "L - Lift"),
        ("Orange", "E - Escalator"),
    ]
    for i, (_, text) in enumerate(legend):
        screen.blit(font.render(text, True, BLACK), (35, y_offset + 15 + i * 15))

def handle_mouse_click(pos):
    global dropdown_open, selected_start, selected_goal, path_result
    x, y = pos
    if 20 <= x <= 220 and 30 <= y <= 60:
        dropdown_open = "start" if dropdown_open != "start" else None
    elif 20 <= x <= 220 and 70 <= y <= 100:
        dropdown_open = "goal" if dropdown_open != "goal" else None
    elif dropdown_open:
        for i in range(min(max_dropdown_display, len(dropdown_options) - dropdown_scroll)):
            if 20 <= x <= 220 and 120 + i * 25 <= y <= 145 + i * 25:
                label = dropdown_options[dropdown_scroll + i]
                if dropdown_open == "start":
                    selected_start = label
                else:
                    selected_goal = label
                dropdown_open = None
                path_result.clear()
                return

def handle_scroll(direction):
    global dropdown_scroll
    if dropdown_open:
        dropdown_scroll = max(0, min(len(dropdown_options) - max_dropdown_display, dropdown_scroll + direction))

def run_pathfinding(start_label, goal_label):
    start = label_to_coord[start_label]
    goal = label_to_coord[goal_label]
    if start == goal:
        return [start]  # Only one dot on itself
    return bidirectional_astar(grid, start, goal)

# Main loop
clock = pygame.time.Clock()
running = True
while running:
    if selected_start and selected_goal and not path_result:
        path_result = run_pathfinding(selected_start, selected_goal)

    draw_grid()
    pygame.display.flip()
    clock.tick(60)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 4:
                handle_scroll(-1)
            elif event.button == 5:
                handle_scroll(1)
            else:
                handle_mouse_click(event.pos)

pygame.quit()
sys.exit()
