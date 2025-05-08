import matplotlib.pyplot as plt


def plot_graph(graph):
    '''Plot the scene graph on the occupancy grid to scale'''
    # find the room nodes
    room_node_idx = graph.room_indices

    rc_idx = room_node_idx + graph.container_indices

    # plot the edge connectivity between rooms and their containers only
    filtered_edges = [
        edge
        for edge in graph.edges
        if edge[1] in rc_idx and edge[0] != 0
    ]

    for (start, end) in filtered_edges:
        p1 = graph.nodes[start]['position']
        p2 = graph.nodes[end]['position']
        x_values = [p1[0], p2[0]]
        y_values = [p1[1], p2[1]]
        plt.plot(x_values, y_values, 'c', linestyle="--", linewidth=0.3)

    # plot room nodes
    for room in rc_idx:
        room_pos = graph.nodes[room]['position']
        room_name = graph.nodes[room]['name']
        plt.text(room_pos[0], room_pos[1], room_name, color='brown',
                 size=6, rotation=40)
