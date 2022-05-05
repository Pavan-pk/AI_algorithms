#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2022, AAIR Lab, ASU"
__authors__ = ["Chirav Dave", "Naman Shah", "Rushang Karia", "Pulkit Verma"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Naman Shah"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import pickle
import goal_formula

def write_objects(fhandle,object_dict,env):
    if env == "bookWorld":
        book_list = object_dict["object"].keys()
        book_loc_list = [book_name + "_iloc" for book_name in book_list]
        bins_list = object_dict["goal"].keys()
        bins_loc_list = [bins_name + "_iloc" for bins_name in bins_list]
        book_list_str = " ".join(book for book in book_list) + " - book"
        book_loc_str = " ".join(book_loc for book_loc in book_loc_list)  + " - location"
        bin_list_str = " ".join(bin_name for bin_name in bins_list) + " - bin"
        bin_loc_str = " ".join(bin_loc for bin_loc in bins_loc_list) + " - location"
        subject_set = set()
        for book in book_list:
            subject_set.add(object_dict["object"][book]["obj_type"].replace(" ","_"))
        subject_str = " ".join(sub_name for sub_name in subject_set) + " - subject"
        fhandle.write("(:objects" + "\n")
        fhandle.write("tbot3 - robot" + "\n")
        fhandle.write("tbot3_init_loc - location" + "\n")
        fhandle.write(book_list_str + "\n")
        fhandle.write(bin_list_str + "\n")
        fhandle.write(book_loc_str + "\n")
        fhandle.write(bin_loc_str + "\n")
        fhandle.write(subject_str + "\n")
        fhandle.write("small large - size" + "\n")
        fhandle.write(")" + "\n")
        object_list = book_list
        object_loc_list = book_loc_list
        goal_list = bins_list
        goal_loc_list = bins_loc_list

    elif env == "cafeWorld":
        food_list = object_dict["object"].keys()
        food_loc_list = [food_name + "_iloc" for food_name in food_list]
        tables_list = object_dict["goal"].keys()
        tables_loc_list = [tables_name + "_iloc" for tables_name in tables_list]
        food_list_str = " ".join(food for food in food_list) + " - food"
        food_loc_str = " ".join(food_loc for food_loc in food_loc_list) + " - location"
        table_list_str = " ".join(table_name for table_name in tables_list) + " - table"
        table_loc_str = " ".join(table_loc for table_loc in tables_loc_list) + " - location"
        subject_set = set()
        for food in food_list:
            subject_set.add(object_dict["object"][food]["obj_type"].replace(" ", "_"))
        subject_str = " ".join(sub_name for sub_name in subject_set) + " - food_type"
        fhandle.write("(:objects" + "\n")
        fhandle.write("tbot3 - robot" + "\n")
        fhandle.write("tbot3_init_loc - location" + "\n")
        fhandle.write(food_list_str + "\n")
        fhandle.write(table_list_str + "\n")
        fhandle.write(food_loc_str + "\n")
        fhandle.write(table_loc_str + "\n")
        fhandle.write(subject_str + "\n")
        fhandle.write("small large - size" + "\n")
        fhandle.write(")" + "\n")
        object_list = food_list
        object_loc_list = food_loc_list
        goal_list = tables_list
        goal_loc_list = tables_loc_list
    else:
        exit(-1)
    return object_list,object_loc_list,goal_list,goal_loc_list

def write_init_state(fhandle,object_dict,object_list,object_loc_list,goal_list,goal_loc_list,env):
    if env == "bookWorld":
        fhandle.write("(:init"+"\n")
        for i in range(len(object_list)):
            fhandle.write("(Book_At {} {})".format(object_list[i],object_loc_list[i]) + "\n")
        for i in range(len(goal_list)):
            fhandle.write("(Bin_At {} {})".format(goal_list[i],goal_loc_list[i]) + "\n")
        for book in object_list:
            fhandle.write("(Book_Subject {} {})".format(book,object_dict["object"][book]["obj_type"].replace(" ","_")) + "\n")
            fhandle.write("(Book_Size {} {})".format(book,object_dict["object"][book]["size"]) + "\n")
        for bin_name in goal_list:
            fhandle.write("(Bin_Subject {} {})".format(bin_name,object_dict["goal"][bin_name]["obj_type"].replace(" ","_")) + "\n")
            fhandle.write("(Bin_Size {} {})".format(bin_name,object_dict["goal"][bin_name]["size"]) + "\n")
        fhandle.write("(Robot_At tbot3 tbot3_init_loc)" + "\n")
        fhandle.write("(Empty_Basket tbot3)"  + "\n")
        fhandle.write(")" + "\n")
    elif env == "cafeWorld":
        fhandle.write("(:init" + "\n")
        for i in range(len(object_list)):
            fhandle.write("(Food_At {} {})".format(object_list[i], object_loc_list[i]) + "\n")
        for i in range(len(goal_list)):
            fhandle.write("(Table_At {} {})".format(goal_list[i], goal_loc_list[i]) + "\n")
        for food in object_list:
            fhandle.write(
                "(Food_Type {} {})".format(food, object_dict["object"][food]["obj_type"].replace(" ", "_")) + "\n")
            fhandle.write("(Portion_Size {} {})".format(food, object_dict["object"][food]["size"]) + "\n")
        for table_name in goal_list:
            fhandle.write("(Ordered {} {})".format(table_name,
                                                         object_dict["goal"][table_name]["obj_type"].replace(" ",
                                                                                                             "_")) + "\n")
            fhandle.write("(Ordered_Portion {} {})".format(table_name, object_dict["goal"][table_name]["size"]) + "\n")
        fhandle.write("(Robot_At tbot3 tbot3_init_loc)" + "\n")
        fhandle.write("(Empty_Basket tbot3)" + "\n")
        fhandle.write(")" + "\n")


def write_pddl(path,object_dict,env):
    if env == "bookWorld":
        fhandle = open(path,"w")
        fhandle.write("(define (problem p01)\n")
        fhandle.write("(:domain bookWorld)\n")
        book_list,book_loc_list,bins_list,bins_loc_list = write_objects(fhandle,object_dict,env)
        write_init_state(fhandle,object_dict,book_list,book_loc_list,bins_list,bins_loc_list,env)
        fhandle.write(goal_formula.get_goal_string(object_dict, book_list,book_loc_list,bins_list,bins_loc_list,env))
    #     fhandle.write(hw2_task.sample_goal_condition(object_dict, book_list,book_loc_list,bins_list,bins_loc_list))
        fhandle.write(")")
        fhandle.close()
    elif env == "cafeWorld":
        fhandle = open(path, "w")
        fhandle.write("(define (problem p01)\n")
        fhandle.write("(:domain cafeWorld)\n")
        food_list, food_loc_list, tables_list, tables_loc_list = write_objects(fhandle, object_dict,env)
        write_init_state(fhandle, object_dict, food_list, food_loc_list, tables_list, tables_loc_list,env)
        fhandle.write(goal_formula.get_goal_string(object_dict, food_list, food_loc_list, tables_list, tables_loc_list,env))
        fhandle.write(")")
        fhandle.close()

