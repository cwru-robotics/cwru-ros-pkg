#include <gtk/gtk.h>
#include <stdio.h>
#include <stdio.h> 
#include <stdlib.h>

void 
on_window_destroy (GtkObject *object, gpointer user_data)
{
        gtk_main_quit();
}

G_MODULE_EXPORT void move_trigger_cb(GtkButton *move_trigger, gpointer data) 
{
  printf("move trigger\n");
  system("rosservice call move_trigger 1");
}


G_MODULE_EXPORT void set_mode0_cb(GtkButton *set_mode0, gpointer data) 
{
  printf("ooh! mode 0!\n");
    system("rosservice call process_mode 0");
}

G_MODULE_EXPORT void set_mode1_cb(GtkButton *set_mode1, gpointer data) 
{
  printf("mode 1\n");
    system("rosservice call process_mode 1");
}

G_MODULE_EXPORT void set_mode2_cb(GtkButton *set_mode2, gpointer data) 
{
  printf("mode 2\n");
    system("rosservice call process_mode 2");
}

G_MODULE_EXPORT void set_mode3_cb(GtkButton *set_mode3, gpointer data) 
{
  printf("mode 3\n");
    system("rosservice call process_mode 3");
}

G_MODULE_EXPORT void set_mode4_cb(GtkButton *set_mode4, gpointer data) 
{
  printf("mode 4\n");
    system("rosservice call process_mode 4");
}


int
main (int argc, char *argv[])
{
        GtkBuilder              *builder;
        GtkWidget               *window;
        
        gtk_init (&argc, &argv);
        
        builder = gtk_builder_new ();
        gtk_builder_add_from_file (builder, "service_btns.glade", NULL);

        window = GTK_WIDGET (gtk_builder_get_object (builder, "window1"));
        gtk_builder_connect_signals (builder, NULL);          
        g_object_unref (G_OBJECT (builder));
        
        gtk_widget_show (window);       
        gtk_main ();
          return 0;
}      
    
