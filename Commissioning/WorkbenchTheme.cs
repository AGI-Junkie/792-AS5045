using System.Windows;
using System.Windows.Controls;
using System.Windows.Markup;
using System.Windows.Media;

namespace TwoLinkCommissioning;

internal static class WorkbenchTheme
{
    public static readonly Brush Ink = Brush("#172B4D"), Muted = Brush("#5D6D83"), Blue = Brush("#245BE8"),
        Teal = Brush("#087F8C"), Red = Brush("#B42338"), Line = Brush("#DFE6F0"), Background = Brush("#F2F5FA");
    public static SolidColorBrush Brush(string color) => (SolidColorBrush)new BrushConverter().ConvertFromString(color)!;
    public static void Apply(FrameworkElement root)
    {
        root.Resources.MergedDictionaries.Add((ResourceDictionary)XamlReader.Parse("""
        <ResourceDictionary xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation" xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml">
          <Style TargetType="Button">
            <Setter Property="Background" Value="White"/><Setter Property="Foreground" Value="#172B4D"/>
            <Setter Property="BorderBrush" Value="#D4DDED"/><Setter Property="BorderThickness" Value="1"/>
            <Setter Property="Padding" Value="12,8"/><Setter Property="Margin" Value="3"/>
            <Setter Property="MinHeight" Value="34"/><Setter Property="FontSize" Value="13"/>
            <Setter Property="Cursor" Value="Hand"/><Setter Property="HorizontalContentAlignment" Value="Center"/>
            <Setter Property="Template"><Setter.Value><ControlTemplate TargetType="Button">
              <Border x:Name="surface" CornerRadius="7" Background="{TemplateBinding Background}" BorderBrush="{TemplateBinding BorderBrush}" BorderThickness="{TemplateBinding BorderThickness}" Padding="{TemplateBinding Padding}">
                <ContentPresenter x:Name="caption" HorizontalAlignment="{TemplateBinding HorizontalContentAlignment}" VerticalAlignment="Center" RecognizesAccessKey="True" TextElement.Foreground="{TemplateBinding Foreground}"/>
              </Border>
              <ControlTemplate.Triggers>
                <Trigger Property="IsMouseOver" Value="True"><Setter TargetName="surface" Property="Opacity" Value="0.85"/></Trigger>
                <Trigger Property="IsPressed" Value="True"><Setter TargetName="surface" Property="Opacity" Value="0.65"/></Trigger>
                <Trigger Property="IsKeyboardFocused" Value="True"><Setter TargetName="surface" Property="BorderBrush" Value="#172B4D"/><Setter TargetName="surface" Property="BorderThickness" Value="2"/></Trigger>
                <Trigger Property="IsEnabled" Value="False"><Setter TargetName="surface" Property="Opacity" Value="0.5"/><Setter TargetName="surface" Property="Background" Value="#E5E7EB"/><Setter TargetName="surface" Property="BorderBrush" Value="#CBD0D8"/><Setter TargetName="caption" Property="TextElement.Foreground" Value="#677386"/><Setter Property="Cursor" Value="Arrow"/></Trigger>
              </ControlTemplate.Triggers>
            </ControlTemplate></Setter.Value></Setter>
          </Style>
          <Style TargetType="TextBox"><Setter Property="FontSize" Value="13"/><Setter Property="Foreground" Value="#172B4D"/><Setter Property="BorderBrush" Value="#CBD5E5"/><Setter Property="Padding" Value="7,5"/><Setter Property="VerticalContentAlignment" Value="Center"/><Setter Property="SelectionBrush" Value="#B2CBFF"/></Style>
          <Style TargetType="ComboBox"><Setter Property="FontSize" Value="13"/><Setter Property="MinHeight" Value="32"/><Setter Property="Padding" Value="5"/><Setter Property="VerticalContentAlignment" Value="Center"/></Style>
          <!-- The default determinate bar also animates its glow. Keep rendering finite so input and heartbeats can run. -->
          <Style TargetType="ProgressBar">
            <Setter Property="Template"><Setter.Value><ControlTemplate TargetType="ProgressBar">
              <Grid>
                <Border x:Name="PART_Track" Background="{TemplateBinding Background}"/>
                <Border x:Name="PART_Indicator" HorizontalAlignment="Left" Background="{TemplateBinding Foreground}"/>
              </Grid>
            </ControlTemplate></Setter.Value></Setter>
          </Style>
          <Style TargetType="TabItem">
            <Setter Property="Padding" Value="14,9"/><Setter Property="FontSize" Value="13"/><Setter Property="Foreground" Value="#172B4D"/>
            <Setter Property="Background" Value="Transparent"/>
            <Setter Property="Template"><Setter.Value><ControlTemplate TargetType="TabItem">
              <Border x:Name="tab" Background="{TemplateBinding Background}" Padding="{TemplateBinding Padding}" BorderThickness="0,0,0,2" BorderBrush="Transparent">
                <ContentPresenter x:Name="tabTitle" ContentSource="Header" VerticalAlignment="Center" TextElement.Foreground="#5D6D83"/>
              </Border>
              <ControlTemplate.Triggers>
                <Trigger Property="IsSelected" Value="True"><Setter TargetName="tabTitle" Property="TextElement.Foreground" Value="#245BE8"/><Setter Property="Background" Value="White"/><Setter TargetName="tab" Property="BorderBrush" Value="#245BE8"/></Trigger>
                <Trigger Property="IsKeyboardFocused" Value="True"><Setter TargetName="tab" Property="BorderBrush" Value="#172B4D"/></Trigger>
              </ControlTemplate.Triggers>
            </ControlTemplate></Setter.Value></Setter>
          </Style>
          <Style TargetType="TabControl"><Setter Property="Background" Value="White"/><Setter Property="BorderBrush" Value="#DFE6F0"/><Setter Property="Padding" Value="8"/></Style>
        </ResourceDictionary>
        """));
    }
    public static TextBlock Text(string text, double size = 13, Brush? color = null) => new() {
        Text = text, FontSize = size, Foreground = color ?? Ink, TextWrapping = TextWrapping.Wrap, Margin = new Thickness(3,4,3,4)
    };
    public static Border Card(UIElement content, Thickness? margin = null) => new() {
        Background = Brushes.White, BorderBrush = Line, BorderThickness = new Thickness(1), CornerRadius = new CornerRadius(10),
        Padding = new Thickness(14), Margin = margin ?? new Thickness(0,0,0,12), Child = content
    };
    public static void Primary(Button b) { b.Background = Blue; b.Foreground = Brushes.White; b.BorderBrush = Blue; }
}
