import heapq
import json


class Dijkstra:
    def __init__(self, field_config):
        self.width = field_config["field_width"]
        self.height = field_config["field_height"]
        self.non_traversable = set(tuple(x) for x in field_config["non_traversable"])

    def neighbors(self, node):
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        result = []
        for dx, dy in directions:
            neighbor = (node[0] + dx, node[1] + dy)
            if (0 <= neighbor[0] < self.width and
                0 <= neighbor[1] < self.height and
                neighbor not in self.non_traversable):
                result.append(neighbor)
        return result

    def dijkstra_search(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))  # (cost, node)
        came_from = {}
        cost_so_far = {start: 0}

        while open_set:
            current_cost, current = heapq.heappop(open_set)

            if current == goal:
                path = []
                while current != start:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            for neighbor in self.neighbors(current):
                new_cost = cost_so_far[current] + 1  # Assumes uniform cost for all edges
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    heapq.heappush(open_set, (new_cost, neighbor))
                    came_from[neighbor] = current

        return None

    # def main(self):
    #     # Load field configuration from JSON file
    #     with open("field_config.json", "r") as f:
    #         field_config = json.load(f)
    #
    #     # Define start and goal nodes
    #     start = (0, 0)
    #     goal = (field_config["field_width"] - 1, field_config["field_height"] - 1)
    #
    #     # Create an instance of Dijkstra and find the path
    #     dijkstra = Dijkstra(field_config)
    #     path = dijkstra.dijkstra_search(start, goal)
    #
    #     if path:
    #         print("Path found:", path)
    #     else:
    #         print("No path found.")
    #
    # if __name__ == "__main__":
    #     main()
    #





