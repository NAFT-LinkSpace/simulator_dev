import openpyxl as xl


wb = xl.Workbook()
ws = wb.active
data = ["a", 1, 2, 3]
wb.save('./test/test.xlsx')
