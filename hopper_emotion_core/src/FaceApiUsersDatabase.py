#!/usr/bin/env python

import os
import sqlite3
import rospy
from hopper_emotion_core.msg import NameAndPersonId
from hopper_emotion_core.srv import GetNameByPersonId, GetPersonIdByName, GetPersonIdByNameResponse, GetNameByPersonIdResponse

class FaceDatabaseAccessor(object):
    def __init__(self):
        rospy.init_node("Face_name_database")
        db_path = os.getenv("HOME") + "/face_database.db"
        database_exits = os.path.isfile(db_path)
        self.database_connection = sqlite3.connect(db_path, check_same_thread=False)
        self.cursor = self.database_connection.cursor()
        if not database_exits:
            self.initialize_database()
        self.new_person_subscriber = rospy.Subscriber("hopper_register_new_person", NameAndPersonId, self.insert_new_person)
        self.get_name_service = rospy.Service("get_name_by_person_id", GetNameByPersonId, self.get_name_by_person_id)
        self.get_person_id_service = rospy.Service("get_person_id_by_name", GetPersonIdByName, self.get_person_id_by_name)
        rospy.spin()


    def initialize_database(self):
        rospy.logwarn("Reinitializing person database")
        self.cursor.execute("DROP TABLE IF EXISTS faces")
        self.cursor.execute("CREATE TABLE faces (name text, personId text)")

    def insert_new_person(self, new_person):
        self.cursor.execute("INSERT INTO faces VALUES (?, ?)", (new_person.name, new_person.person_id))
        self.database_connection.commit()

    def get_name_by_person_id(self, msg):
        self.cursor.execute("SELECT * FROM faces WHERE personId IS (?)", (msg.person_id, ))
        result = self.cursor.fetchone()
        if result is not None:
            return GetNameByPersonIdResponse(result[0])
        else:
            return GetNameByPersonIdResponse("")

    def get_person_id_by_name(self, msg):
        self.cursor.execute("SELECT * FROM faces WHERE name IS (?)", (msg.name, ))
        result = self.cursor.fetchone()
        if result is not None:
            return GetPersonIdByNameResponse(result[1])
        else:
            return GetPersonIdByNameResponse("")


if __name__ == "__main__":
    FaceDatabaseAccessor()