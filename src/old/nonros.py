import csv

path_file = '/home/sidd/paths/path1.csv'

path_list = list()

with open(path_file, 'rb') as fr:
  reader = csv.reader(fr, delimiter=',')
  for row in reader:
    path_list.append(row)
    import pdb; pdb.set_trace()
