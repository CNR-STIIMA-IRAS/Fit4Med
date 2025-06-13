import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sqlite3
import numpy as np
import os
import sys

def extract_topic_data(bag_name, topic_name, min_fields=3):
    db_file = os.path.join(f'/home/fit4med/bag_records/{bag_name}', 'data.db3')
    if not os.path.exists(db_file):
        raise FileNotFoundError(f" {bag_name} non contiene 'data.db3'.")

    conn = sqlite3.connect(db_file)
    cursor = conn.cursor()

    topics = cursor.execute("SELECT id, name, type FROM topics").fetchall()
    topic_map = {name: (id, typ) for id, name, typ in topics}

    if topic_name not in topic_map:
        raise ValueError(f" Topic '{topic_name}' non trovato nella bag.")

    topic_id, topic_type = topic_map[topic_name]
    MsgClass = get_message(topic_type)

    data_x, data_y, data_z = [], [], []
    timestamps = []

    for timestamp, raw in cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id={topic_id}"):
        msg = deserialize_message(raw, MsgClass)

        # Gestione messaggi
        if topic_name == "/joint_states":
            if hasattr(msg, "position") and len(msg.position) >= min_fields:
                data_x.append(msg.position[0])
                data_y.append(msg.position[1])
                data_z.append(msg.position[2])
                timestamps.append(timestamp * 1e-9)
        elif topic_name == "/tf":
            for transform in msg.transforms:
                child = transform.child_frame_id
                if child.startswith("tag"):  # Apriltag generico
                    t = transform.transform.translation
                    data_x.append(t.x)
                    data_y.append(t.y)
                    data_z.append(t.z)
                    timestamps.append(timestamp * 1e-9)
        else:
            pass  # estensibile

    conn.close()
    print(f"✔️ Estratti {len(data_x)} messaggi da '{topic_name}' nella bag '{bag_path}'.")
    return np.array(data_x), np.array(data_y), np.array(data_z), np.array(timestamps)


def main():
    if len(sys.argv) != 3:
        print("Usage: python extract_joint_and_tf_from_two_bags.py <bag_joint_states> <bag_tf>")
        sys.exit(1)

    bag_joint = sys.argv[1]
    bag_tf = sys.argv[2]

    try:
        jx, jy, jz, jt = extract_topic_data(bag_joint, "/joint_states")
        tx, ty, tz, tt = extract_topic_data(bag_tf, "/tf")
    except Exception as e:
        print(e)
        sys.exit(1)

    # Esempio di stampa
    print("\nEsempio dati:")
    print(f"Joint X[0]: {jx[0] if len(jx) else 'n/a'}")
    print(f"TF X[0]:    {tx[0] if len(tx) else 'n/a'}")

if __name__ == "__main__":
    main()
