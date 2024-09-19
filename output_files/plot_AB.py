
import os, sys
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib import rc

plt.rcParams["grid.color"] = "lightgray"
plt.rcParams["grid.linewidth"] = 0.5
matplotlib.rc("font", family="serif", size=9)
matplotlib.rcParams["text.usetex"] = True
rc('text', usetex=True)


def main():
    fileName = "matrices.csv"

    # model_dir = os.path.join(os.getcwd(), "trained_models", model_name)

    df = pd.read_csv("output_files/" + fileName, delimiter=",")
    n = len(df)

    X = df[['x1','x2','x3','x4']].to_numpy()
    Y = df[['y1','y2']].to_numpy()
    A = df[['A11','A12','A21','A22']].to_numpy().reshape(n, 2, 2)   
    B = df[['A11','A21']].to_numpy().reshape(n, 2, 1)   

    plot_A_Matrix(Y, A, os.path.dirname(__file__), True)

    plot_B_Matrix(Y, B, os.path.dirname(__file__), True)
    
def plot_A_Matrix(Yp, A, save_dir, save=False):
    print("Plotting A Matrix ...")

    lifted_states_name = [
        "$y_1$",
        "$y_2$",
        "$y_3$",
        "$y_4$",
        "$y_5$",
        "$y_6$",
    ]  # Example state names

    fig, axs = plt.subplots(2, 2, figsize=(7, 6))

    sc1 = axs[0, 0].scatter(
        Yp[:, 0].flatten(),
        Yp[:, 1].flatten(),
        c=A[:, 0, 0].flatten(),
        cmap="viridis",
        s=2,
    )
    sc2 = axs[0, 1].scatter(
        Yp[:, 0].flatten(),
        Yp[:, 1].flatten(),
        c=A[:, 0, 1].flatten(),
        cmap="viridis",
        s=2,
    )
    sc3 = axs[1, 0].scatter(
        Yp[:, 0].flatten(),
        Yp[:, 1].flatten(),
        c=A[:, 1, 0].flatten(),
        cmap="viridis",
        s=2,
    )
    sc4 = axs[1, 1].scatter(
        Yp[:, 0].flatten(),
        Yp[:, 1].flatten(),
        c=A[:, 1, 1].flatten(),
        cmap="viridis",
        s=2,
    )

    # Set the grid to appear behind plot elements
    for ax in axs.flat:
        ax.set_axisbelow(True)
        ax.grid(which="both", color="lightgray", linestyle="--", linewidth=0.5)

    # Add color bars for each scatter plot
    cbar_kwargs = {"fraction": 0.046, "pad": 0.04}  # Adjust these values as needed
    fig.colorbar(sc1, ax=axs[0, 0], **cbar_kwargs)
    fig.colorbar(sc2, ax=axs[0, 1], **cbar_kwargs)
    fig.colorbar(sc3, ax=axs[1, 0], **cbar_kwargs)
    fig.colorbar(sc4, ax=axs[1, 1], **cbar_kwargs)

    axs[0, 0].set_title("$A_{1,1}$")
    axs[0, 0].set_xlabel(lifted_states_name[0])
    axs[0, 0].set_ylabel(lifted_states_name[1])
    axs[0, 0].minorticks_on()
    # axs[0, 0].set_aspect("equal", adjustable="box")

    axs[0, 1].set_title("$A_{1,2}$")
    axs[0, 1].set_xlabel(lifted_states_name[0])
    axs[0, 1].set_ylabel(lifted_states_name[1])
    axs[0, 1].minorticks_on()
    # axs[0, 1].set_aspect("equal", adjustable="box")

    axs[1, 0].set_title("$A_{2,1}$")
    axs[1, 0].set_xlabel(lifted_states_name[0])
    axs[1, 0].set_ylabel(lifted_states_name[1])
    axs[1, 0].minorticks_on()
    # axs[1, 0].set_aspect("equal", adjustable="box")

    axs[1, 1].set_title("$A_{2,2}$")
    axs[1, 1].set_xlabel(lifted_states_name[0])
    axs[1, 1].set_ylabel(lifted_states_name[1])
    axs[1, 1].minorticks_on()
    # axs[1, 1].set_aspect("equal", adjustable="box")

    plt.tight_layout()  # Adjust layout to make room for the colorbar
    plt.draw()
    plt.pause(0.1)
    if save:
        # plt.savefig(os.path.join(save_dir, "A_matrix.pdf"), format="pdf")
        plt.savefig(os.path.join(save_dir, "A_matrix.png"), format="png", dpi=300)


def plot_B_Matrix(Yp, B, save_dir, save=False):
    print("Plotting B Matrix ...")

    lifted_states_name = [
        "$y_1$",
        "$y_2$",
        "$y_3$",
        "$y_4$",
        "$y_5$",
        "$y_6$",
    ]  # Example state names

    fig, axs = plt.subplots(1, 2, figsize=(7, 3))

    sc1 = axs[0].scatter(
        Yp[:, 0].flatten(),
        Yp[:, 1].flatten(),
        c=B[:, 0, 0].flatten(),
        cmap="viridis",
        s=2,
    )
    sc2 = axs[1].scatter(
        Yp[:, 0].flatten(),
        Yp[:, 1].flatten(),
        c=B[:, 1, 0].flatten(),
        cmap="viridis",
        s=2,
    )
    # sc3 = axs[0, 1].scatter(
    #     Yp[:, :, 0].flatten(),
    #     Yp[:, :, 1].flatten(),
    #     c=B[:, :, 2].flatten(),
    #     cmap="viridis",
    #     s=2,
    # )
    # sc4 = axs[1, 1].scatter(
    #     Yp[:, :, 0].flatten(),
    #     Yp[:, :, 1].flatten(),
    #     c=B[:, :, 3].flatten(),
    #     cmap="viridis",
    #     s=2,
    # )

    # Set the grid to appear behind plot elements
    for ax in axs.flat:
        ax.set_axisbelow(True)
        ax.grid(which="both", color="lightgray", linestyle="--", linewidth=0.5)

    # Add color bars for each scatter plot
    cbar_kwargs = {"fraction": 0.046, "pad": 0.04}  # Adjust these values as needed
    fig.colorbar(sc1, ax=axs[0], **cbar_kwargs)
    fig.colorbar(sc2, ax=axs[1], **cbar_kwargs)
    # fig.colorbar(sc3, ax=axs[0, 1], **cbar_kwargs)
    # fig.colorbar(sc4, ax=axs[1, 1], **cbar_kwarg
    axs[0].set_title("$B_{1,1}$")
    axs[0].set_xlabel(lifted_states_name[0])
    axs[0].set_ylabel(lifted_states_name[1])
    axs[0].minorticks_on()
    # axs[0].set_aspect("equal", adjustable="box")

    axs[1].set_title("$B_{2,1}$")
    axs[1].set_xlabel(lifted_states_name[0])
    axs[1].set_ylabel(lifted_states_name[1])
    axs[1].minorticks_on()
    # axs[1].set_aspect("equal", adjustable="box")

    # axs[0, 1].set_title("$B_{3,1}$")
    # axs[0, 1].set_xlabel(lifted_states_name[0])
    # axs[0, 1].set_ylabel(lifted_states_name[1])
    # axs[0, 1].minorticks_on()
    # axs[0, 1].set_aspect("equal", adjustable="box")

    # axs[1, 1].set_title("$B_{4,1}$")
    # axs[1, 1].set_xlabel(lifted_states_name[0])
    # axs[1, 1].set_ylabel(lifted_states_name[1])
    # axs[1, 1].minorticks_on()
    # axs[1, 1].set_aspect("equal", adjustable="box")

    plt.tight_layout()  # Adjust layout to make room for the colorbar
    plt.draw()
    plt.pause(0.1)
    if save:
        # plt.savefig(os.path.join(save_dir, "B_matrix.pdf"), format="pdf")
        plt.savefig(os.path.join(save_dir, "B_matrix.png"), format="png", dpi=300)

if __name__ == "__main__":
    main()
