#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Using the magic encoding


import rospy
import pylab
import graph_generator

import numpy
import matplotlib
import matplotlib.pyplot as plt

import os


def get_mean(path, file_name):
    my_file = open ( path + file_name , 'r')
    data = [ map(float, line[:-2].split(' ')) for line in my_file ]

    for i in range(len(data[0])):
        data_column = [row[i] for row in data]
        print numpy.mean(data_column[2700:])

        fit = pylab.polyfit(range(len(data_column)), data_column, 2)
        


def get_score_mean(path, file_name):
    my_file = open ( path + file_name , 'r')
    data = [ map(float, line.split(' ')) for line in my_file ]

    for i in range(len(data[0])):
        data_column = [row[i] for row in data]
        print 'score ', numpy.mean(data_column[2700:])

        fit = pylab.polyfit(range(len(data_column)), data_column, 2)
        print fit


def create_score_graph(save_path, path, file_name):
    my_file = open ( path + file_name , 'r')
    data = [ map(float, line.split(' ')) for line in my_file ]

    figure = plt.figure(figsize=(8.0, 5.0))
    ax = figure.add_subplot(111)
    # plots = []

    matplotlib.rcParams['lines.markersize'] = 0.5*matplotlib.rcParams['lines.markersize']
    matplotlib.rcParams['lines.linewidth'] = 2*matplotlib.rcParams['lines.linewidth']

    for i in range(len(data[0])):
        data_column = [row[i] for row in data[:3000]]
        fit = pylab.polyfit(range(len(data_column)), data_column, 4)
        

        fit_fn = graph_generator.gen_polinomial_function(fit) # pylab.poly1d(fit)

        # ax.plot(range(len(data_column)), data_column, '.', range(len(data_column)), fit_fn(data_column), '--')
        p, = ax.plot(data[:3000], 'b.')
        p, = ax.plot(range(len(data_column)), fit_fn(range(len(data_column))), 'r--')
        #plots.append(p)
        #plt.show()

        # ax.legend(plots, ['Pontuacao da Partida'])
        figure.suptitle(u'Pontuação da Partida')
        ax.set_xlabel('Partida')
        ax.set_ylabel(u"Pontuação")
        save_file_name = save_path + 'match_scores____pol.eps'
        figure.savefig(save_file_name, dpi=400)

        plt.close(figure)

    matplotlib.rcParams['lines.markersize'] = 2*matplotlib.rcParams['lines.markersize']
    matplotlib.rcParams['lines.linewidth'] = 0.5*matplotlib.rcParams['lines.linewidth']


def create_chosen_behaviors_graph(save_path, path, file_name):
    my_file = open ( path + file_name , 'r')
    data = [ map(float, line[:-2].split(' ')) for line in my_file ]

    figure = plt.figure(figsize=(8.0, 5.0))
    ax = figure.add_subplot(111)
    ax.plot(data, '.')

    matplotlib.rcParams['lines.markersize'] = 0.25*matplotlib.rcParams['lines.markersize']

    ax.axis([0, 3000, 0, 550])
    ax.legend(['Ficar parado', 'Comer', 'Fugir'])
    figure.suptitle(u'Comportamentos escolhidos')
    ax.set_xlabel('Partida')
    ax.set_ylabel(u"Número de vezes esolhido")
    save_file_name = save_path + 'chosen_behaviors.eps'
    figure.savefig(save_file_name, dpi=400)

    plt.close(figure)

    matplotlib.rcParams['lines.markersize'] = 4*matplotlib.rcParams['lines.markersize']


def create_chosen_behaviors_pol_graph(save_path, path, file_name):
    my_file = open ( path + file_name , 'r')
    data = [ map(float, line[:-2].split(' ')) for line in my_file ]

    figure = plt.figure(figsize=(8.0, 5.0))
    ax = figure.add_subplot(111)
    # p, = ax.plot(data, '.')
    plots = []

    matplotlib.rcParams['lines.linewidth'] = 2*matplotlib.rcParams['lines.linewidth']

    for i in range(len(data[0])):
        data_column = [row[i] for row in data]
        fit = pylab.polyfit(range(len(data_column)), data_column, 3)
        fit_fn = graph_generator.gen_polinomial_function(fit)

        p, = ax.plot(range(len(data_column)), fit_fn(range(len(data_column))), '-')
        plots.append(p)
        #plt.show()

    ax.axis([0, 3000, -10, 100])
    ax.legend(plots, ['Ficar parado', 'Comer', 'Fugir'])
    figure.suptitle(u'Comportamentos escolhidos')
    ax.set_xlabel('Partida')
    ax.set_ylabel(u"Número de vezes esolhido")
    save_file_name = save_path + 'chosen_behaviors____pol.eps'
    figure.savefig(save_file_name, dpi=400)

    plt.close(figure)

    matplotlib.rcParams['lines.linewidth'] = 0.5*matplotlib.rcParams['lines.linewidth']


def create_chosen_behaviors_pol_graph2(save_path, path, file_name):
    my_file = open ( path + file_name , 'r')
    data = [ map(float, line[:-2].split(' ')) for line in my_file ]

    # p, = ax.plot(data, '.')
    if len(data[0]) == 3:
        behaviors = ['Ficar parado', 'Comer', 'Fugir']
    else:
        behaviors = ['Ficar parado', 'Comer', 'Fugir', u'Comer Capsula', u'Caçar']

    matplotlib.rcParams['lines.linewidth'] = 2*matplotlib.rcParams['lines.linewidth']

    for i in range(len(data[0])):
        figure = plt.figure(figsize=(8.0, 5.0))
        ax = figure.add_subplot(111)

        data_column = [row[i] for row in data]
        fit = pylab.polyfit(range(len(data_column)), data_column, 3)
        fit_fn = graph_generator.gen_polinomial_function(fit)

        ax.plot(data_column, 'b.')
        p, = ax.plot(range(len(data_column)), fit_fn(range(len(data_column))), 'r-')

        ax.axis([0, 3000, -10, 600])
        ax.legend(['Amostras', u'Polinômio'])
        figure.suptitle(u'Comportamento ' + behaviors[i])
        ax.set_xlabel('Partida')
        ax.set_ylabel(u"Número de vezes esolhido")
        save_file_name = save_path + 'chosen_behaviors____pol__' + str(i) + '.eps'
        figure.savefig(save_file_name, dpi=400)

    plt.close(figure)

    matplotlib.rcParams['lines.linewidth'] = 0.5*matplotlib.rcParams['lines.linewidth']


def create_per_match_weights_graph(save_path, path, file_name):
    if not hasattr(create_per_match_weights_graph, "counter"):
        create_per_match_weights_graph.counter = 0  # it doesn't exist yet, so initialize it
    else:
        create_per_match_weights_graph.counter += 1

    my_file = open ( path + file_name , 'r')
    data = [ map(float, line[:-2].split(' ')) for line in my_file ]

    counter = create_per_match_weights_graph.counter

    behaviors = ['Ficar parado', 'Comer', 'Fugir', u'Comer Capsula', u'Caçar']
    behaviors_ascii = ['Ficar parado', 'Comer', 'Fugir', u'Comer Capsula', u'Cacar']
    behavior = behaviors[counter]
    behavior_ascii = behaviors_ascii[counter]
    weights = [u'Bias', u'Dist. Comida', u'Dist. Capsula', u'Prob. Existir Capsula', u'Prob. Existir Fant. Branco', 
                                u'Prob. Fantasma Perto', u'Prob. Fantasma Branco Perto']

    figure = plt.figure(figsize=(8.0, 5.0))
    ax = figure.add_subplot(111)
    ax.plot(data, '-')

    matplotlib.rcParams['lines.markersize'] = 0.25*matplotlib.rcParams['lines.markersize']

    ax.axis([0, 3000, -250, 150])
    ax.legend(weights)
    figure.suptitle(u'Pesos ' + behavior)
    ax.set_xlabel('Partida')
    ax.set_ylabel(u"Valor")
    save_file_name = save_path + 'per_match_weights__' + behavior_ascii + '.eps'
    figure.savefig(save_file_name, dpi=400)

    plt.close(figure)

    matplotlib.rcParams['lines.markersize'] = 4*matplotlib.rcParams['lines.markersize']

    return data



def create_weights_graph(save_path, weights_data):

    if len(weights_data[0]) == 3:
        behaviors = ['Ficar parado', 'Comer', 'Fugir']
        weights = ['Bias', 'Dist. Comida', 'Prob. Fantasma']
    else:
        behaviors = ['Ficar parado', 'Comer', 'Fugir', 'Comer Capsula', u'Caçar']
        weights = ['Bias', 'Dist. Comida', 'Dist. Capsula', 'Prob. Existir Capsula', 'Prob. Existir Fant. Branco', 'Prob. Fantasma Perto', 'Prob. Fantasma Branco Perto']
        

    matplotlib.rcParams['lines.markersize'] = 0.25*matplotlib.rcParams['lines.markersize']

    for i in range(len(weights_data[0][0])):
        figure = plt.figure(figsize=(8.0, 5.0))
        ax = figure.add_subplot(111)

        single_weight_column = []

        for behavior_data in weights_data:
            behavior_weight_data = [row[i] for row in behavior_data]
            single_weight_column.append(behavior_weight_data)

        single_weight_column = map(list, zip(*single_weight_column))

        ax.plot(single_weight_column, '-')

        if i == 1 and len(behaviors) == 3:
            ax.axis([0, 3000, -13, 5])
        ax.legend(behaviors)
        figure.suptitle(u'Comportamento ' + weights[i])
        ax.set_xlabel('Partida')
        ax.set_ylabel(u"Valor")
        save_file_name = save_path + 'weights____pol__' + str(weights[i]) + '.eps'
        figure.savefig(save_file_name, dpi=400)

    matplotlib.rcParams['lines.markersize'] = 4*matplotlib.rcParams['lines.markersize']

def create_graphs_directory(path):
    directory = path + 'final_graphs/'
    if not os.path.exists(directory):
        os.makedirs(directory)
    return directory

def get_data():
    path = graph_generator.get_path_to_package('log_13/')
    save_path = create_graphs_directory(path)
    log_files = graph_generator.get_log_files(path)
    log_files.sort()

    weight_data = []

    for log_file in log_files:
        if log_file.endswith('.txt'):
            if log_file.startswith('match_behaviors_'):
                get_mean(path, log_file)
                create_chosen_behaviors_graph(save_path, path, log_file)
                create_chosen_behaviors_pol_graph(save_path, path, log_file)
                create_chosen_behaviors_pol_graph2(save_path, path, log_file)
            if log_file.startswith('match_scores_'):
                get_score_mean(path, log_file)
                create_score_graph(save_path, path, log_file)
            if log_file.startswith('per_match_behavior'):
                behavior_data = create_per_match_weights_graph(save_path, path, log_file)
                weight_data.append(behavior_data)

    create_weights_graph(save_path, weight_data)


if __name__ == '__main__':
    try:
        get_data()
    except rospy.ROSInterruptException: pass