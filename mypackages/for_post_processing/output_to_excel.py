import pandas as pd
from openpyxl import load_workbook
from openpyxl.styles import Font
from pathlib import Path

def create_history_excel(output):
    # Create a Pandas Excel writer using XlsxWriter as the engine
    with pd.ExcelWriter('output.xlsx', engine='openpyxl') as writer:
        for key, value in output.items():
            # Skip the 'Summary' key for the Excel file
            if key == 'Summary':
                continue
            # Convert the dictionary to a DataFrame
            df = pd.DataFrame(value)
            # Write the DataFrame to an Excel sheet with the name of the key, without the index
            df.to_excel(writer, sheet_name=key, index=False)

def flatten_dict(dd, separator='_', prefix=''):
    return { prefix + separator + k if prefix else k : v
             for kk, vv in dd.items()
             for k, v in flatten_dict(vv, separator, kk).items()
             } if isinstance(dd, dict) else { prefix : dd }

def create_summary_csv(output, path):
    # Flatten the 'Summary' data
    flat_summary = flatten_dict(output['Summary'])
    # Convert the flattened data to a DataFrame
    summary_df = pd.DataFrame(flat_summary, index=[0])
    # Write the DataFrame to a CSV file
    summary_df.to_csv('summary.csv', index=False)


def main(output):
    create_summary_csv(output)
    create_history_excel(output)
    
if __name__ == "__main__":
    import sys
    from pathlib import Path
    sys.path.append(str(Path(__file__).parent.parent.parent))
    print(sys.path)
    from mypackages.for_main_processing import solve_ode_full_output

    output = solve_ode_full_output.solve_all(3, 180)
    
    main(output)