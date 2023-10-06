import pygame
import sys

# Initialize pygame
pygame.init()

# Colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

# Screen dimensions
WIDTH, HEIGHT = 800, 600

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption('Graph Theory Visualizer')

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.color = RED
        self.radius = 10

    def draw(self):
        pygame.draw.circle(screen, self.color, (self.x, self.y), self.radius)

    def is_clicked(self, x, y):
        return (self.x - x) ** 2 + (self.y - y) ** 2 <= self.radius ** 2

nodes = []
edges = []

running = True
while running:
    screen.fill(WHITE)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.MOUSEBUTTONDOWN:
            x, y = pygame.mouse.get_pos()
            clicked_nodes = [node for node in nodes if node.is_clicked(x, y)]
            
            if not clicked_nodes:  # If no node is clicked, create a new node
                nodes.append(Node(x, y))
            elif len(clicked_nodes) == 1:
                start_node = clicked_nodes[0]
                while pygame.mouse.get_pressed()[0]:  # Left button held down
                    end_x, end_y = pygame.mouse.get_pos()
                    pygame.draw.line(screen, BLUE, (start_node.x, start_node.y), (end_x, end_y), 2)
                    pygame.display.flip()
                    for e in pygame.event.get():
                        if e.type == pygame.MOUSEBUTTONUP:
                            end_nodes = [node for node in nodes if node.is_clicked(end_x, end_y)]
                            if end_nodes:
                                end_node = end_nodes[0]
                                edges.append((start_node, end_node))

    for node in nodes:
        node.draw()
    for edge in edges:
        pygame.draw.line(screen, BLUE, (edge[0].x, edge[0].y), (edge[1].x, edge[1].y), 2)

    pygame.display.flip()

pygame.quit()
sys.exit()
