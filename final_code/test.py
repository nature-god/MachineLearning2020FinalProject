from Test import ResultAnalysis

#计算准确率并画图
#最后一行为Wu=none时的结果
resFileList = ['./data/result_Sample_30_WU_1.txt',
               './data/result_Sample_30_WU_10.txt',
               './data/result_Sample_30_WU_100.txt',
               './data/result_Sample_30_WU_1000.txt',
               './data/result_raw_Length_30.txt']
#画折线图
res = ResultAnalysis([1,10,100,1000], resFileList, './data/road_network.txt','./data/ground_truth_route.txt').DrawLineGraph()
#画柱状图
#res2 = ResultAnalysis(['1','10','100','1000'], resFileList, './data/road_network.txt','./data/ground_truth_route.txt').DrawBarGraph()