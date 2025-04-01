from ultralytics import YOLO
import json 

def main():

    json_file = '/home/lucap/code/fresco_assembly_sequence/fresco_assembly_sequence/int_week_placements.json'
    with open(json_file, 'r') as jf:
        placements = json.load(jf)
    test_img_path = '/media/lucap/big_data/datasets/repair/recognition_benchmark/realsense_camera/frame2_Color.png'
    model_path = '/home/lucap/code/rp-playground/recognition/runs/obb/global_recognition_06_03/weights/best.pt'
    model = YOLO(model_path)
    out = model(test_img_path)
    breakpoint()
    print(f'\nfound {len(out[0].obb)} fragments!\n')
    for obb in out[0].obb:
        xywhr = obb.xywhr[0]
        xc = obb.xywhr[0][0].item()
        yc = obb.xywhr[0][1].item()
        name = out[0].names[obb.cls.item()]
        group = name[name.index('G')+1:]
        fragment_name = name[:name.index('G')-1]
        fragment_name = fragment_name.split('_')[0] + '_00' + fragment_name.split('_')[1]
        assembly_position = placements[f'group_{group}'][f'{fragment_name}_intact_mesh']
        print(f"found {fragment_name} (group {group}) with center in {xc:.2f}, {yc:.2f} (pixel coordinates). \nIt should be placed (in real world coordaintes) in: {assembly_position['trans_x']}, {assembly_position['trans_y']}\n rotated by {assembly_position['ori_yaw']}")       


if __name__ == '__main__':

    main()