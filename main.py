from Map import Map
from planner.coverage_planner import CoveragePlanner


def show_graph(grid_map, path_obj):
    import matplotlib.pyplot as plt

    plt.figure(1)

    occ_x, occ_y = [], []
    for i in range(0, grid_map.width + 1):
        for j in range(0, grid_map.height + 1):
            if grid_map.occ_mask[i, j]:
                occ_x.append(i * grid_map.robot_len + grid_map.min_x)
                occ_y.append(j * grid_map.robot_len + grid_map.min_y)
    plt.plot(occ_x, occ_y, '.k', label='obstacle')

    mode_color = {'cover': 'y', 'replan': 'r', 'avoid': 'b'}
    for mode in mode_color:
        xs, ys = [], []
        for x, y, _, m in path_obj:
            if m == mode:
                xs.append(x * grid_map.robot_len + grid_map.min_x)
                ys.append(y * grid_map.robot_len + grid_map.min_y)
        if xs:
            plt.plot(xs, ys, '.-', color=mode_color[mode], label=mode)

    plt.legend()
    plt.show()


def main():
    map_path = 'v2.smap'
    my_map = Map()
    my_map.initialize_map(map_path)

    planner = CoveragePlanner(my_map)

    # 主链路：map -> global_coverage -> hole_fill -> runtime_avoid -> metrics
    path_obj = planner.plan(start_world=(20, 10), dynamic_obstacles=[])

    print(f'coverage_ratio: {planner.coverage_ratio:.4f}')
    print(f'redundant_ratio: {planner.redundant_ratio:.4f}')
    print(f'uncovered_components: {len(planner.uncovered_components)}')
    print(f'path_length: {len(path_obj)}')

    show_graph(my_map, path_obj)


if __name__ == '__main__':
    main()
