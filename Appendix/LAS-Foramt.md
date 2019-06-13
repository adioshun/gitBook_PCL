
# LAS 포맷

- [정의](https://www.asprs.org/committee-general/laser-las-file-format-exchange-activities.html)
- the most commonly used binary point data exchange format
- The LAS file format is a public file format for the interchange of 3-dimensional point cloud data data between data users
- supports the exchange of any 3-dimensional x,y,z tuplet
- [What Lidar processing tools are available in Python?](https://gis.stackexchange.com/questions/88322/what-lidar-processing-tools-are-available-in-python)
- 항공용 Lidar 데이터에서 주로 사용하는지 확인 필요

# pcap to LAS

> LAS format - the most commonly used binary point data exchange format

- (1) I suggest you create many many txt files containing the points in xyz or xyzi a layout (i = intensity). Maybe one file per driveline. (벨로뷰의 csv저장 기능 활용)
- (2) Then you convert each of them to the LAZ format with txt2las using the '-parse xyz' or the '-parse xyzi' option.
- (3) Then you tile the resulting LAZ files into buffered tiles that have less than, say 10 million, points per tile with lastile. `lastile -v -i drivelines/*.laz -merged -tile_size 100 -buffer 5 -odir raw_tiles -o gmu.laz`
- (4) Then you refine tiles to 10 million points per tile if needed
`lastile -v -i raw_tiles/*.laz -refine_tiles 10000000 -cores 4`
- (5) And then - starting from the tiles in the tiles_raw folder - you run one of those many tile-based batch processing pipelines for ground classification and DTM generation described previously in numerous tutorials, videos, and forum entries.









